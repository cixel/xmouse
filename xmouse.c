#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/usb/input.h>
#include <linux/leds.h>

// the kernel whines and locks certain priveleges unless the license is specified
MODULE_LICENSE("GPL");
MODULE_AUTHOR("the internet");
MODULE_DESCRIPTION("mouse driver for the Microsoft Xbox 360 controller");
MODULE_SUPPORTED_DEVICE("Microsoft Xbox 360 controller");

#define XMOUSE_PKT_LEN 32 // 20 bit input, 12 bit output

struct usb_xmouse {
    struct input_dev *dev;	/* input device interface */
    struct usb_device *udev;	/* usb device */
    struct usb_interface *intf;	/* usb interface */

    int pad_present;

    struct urb *irq_in;		/* urb for interrupt in report */
    unsigned char *idata;	/* input data */
    dma_addr_t idata_dma;

    struct urb *irq_out;	/* urb for interrupt out report */
    unsigned char *odata;	/* output data */
    dma_addr_t odata_dma;
    struct mutex odata_mutex;

    struct xmouse_led *led;
    
    char phys[64];		/* physical device path */
};

static const signed short xmouse_abs[] = { ABS_X, ABS_Y, -1 };
static const signed short xmouse_btns[] = { BTN_LEFT, BTN_RIGHT, -1};

static void xmouse_packet(struct usb_xmouse *xmouse, u16 cmd, unsigned char *data) {
    struct input_dev *dev = xmouse->dev; // pointer to the physical interface

    /* http://free60.org/GamePad contains the packet information I used  */

    /* A and B -> left and right mouse button */
    /* data[3] = 00000000                     */ 
    /*           ||||||||                     */
    /*		 |||||||--BTN_TL   + 00000001 */ 
    /*		 ||||||---BTN_TR   + 00000010 */
    /*		 |||||----BTN_MODE + 00000100 */
    /*		 ||||-----unused   + 00001000 */
    /*		 |||------BTN_A	   + 00010000 */
    /*		 ||-------BTN_B    + 00100000 */
    /*		 |--------BTN_X    + 01000000 */
    /*		 ---------BTN_Y    + 10000000 */
    input_report_key(dev, BTN_LEFT,  data[3] & 0x10); // A
    input_report_key(dev, BTN_RIGHT, data[3] & 0x20); // B

    /* left stick -> mouse coordinates */
    /* data[6] and data[8]*/
    /* casting to read each set of 16 bits as a signed little-endian integer*/
    input_report_abs(dev, ABS_X,  (__s16) le16_to_cpup((__le16 *)(data + 6)));
    input_report_abs(dev, ABS_Y, ~(__s16) le16_to_cpup((__le16 *)(data + 8)));
}

// --------------output/input interrupt and inits--------------------
static void xmouse_irq_in(struct urb *urb){
    struct usb_xmouse *xmouse = urb->context;
    struct device *dev = &xmouse->intf->dev;
    int retval, status;

    status = urb->status;
    switch(status) {
    case 0:
	break; // yay
    case -ECONNRESET: // these are all predefined macros containing some value for what type of error has occured
    case -ENOENT:
    case -ESHUTDOWN:
	return;
    default:
	goto exit;
    }

    // iff the interrupt request for input succeeds, call:
    xmouse_packet(xmouse, 0, xmouse->idata);
    // ... which calls input events

exit:
    retval = usb_submit_urb(urb, GFP_ATOMIC);
    if (retval)
	dev_err(dev, "%s - usb_submit_urb failed with result %d\n", __func__, retval);
}

static void xmouse_irq_out(struct urb *urb){
    struct usb_xmouse *xmouse = urb->context;
    struct device *dev = &xmouse->intf->dev;
    int retval, status;

    status = urb->status;
    switch(status) {
    case 0:
	/* yay */
	return; 
    case -ECONNRESET:
    case -ENOENT:
    case -ESHUTDOWN:
	dev_dbg(dev, "%s - urb shutting down with status: %d\n", __func__, status);
	return;
    default:
	dev_dbg(dev, "%s - nonzero urb status received: %d\n", __func__, status);
	goto exit;
    }

exit:
    retval = usb_submit_urb(urb, GFP_ATOMIC);
    if (retval)
	dev_err(dev, "%s - usb_submit_urb failed with result %d\n", __func__, retval);
}
static int xmouse_init_output(struct usb_interface *intf, struct usb_xmouse *xmouse){
    struct usb_endpoint_descriptor *ep_irq_out;
    int error;

    xmouse->odata = usb_alloc_coherent(xmouse->udev, XMOUSE_PKT_LEN, GFP_KERNEL, &xmouse->odata_dma);
    if (!xmouse->odata) {
	    error = -ENOMEM;
	    goto fail1;
    }

    mutex_init(&xmouse->odata_mutex);

    xmouse->irq_out = usb_alloc_urb(0, GFP_KERNEL);
    if (!xmouse->irq_out) {
	    error = -ENOMEM;
	    goto fail2;
    }

    ep_irq_out = &intf->cur_altsetting->endpoint[1].desc;
    usb_fill_int_urb(xmouse->irq_out, xmouse->udev,
		     usb_sndintpipe(xmouse->udev, ep_irq_out->bEndpointAddress),
		     xmouse->odata, XMOUSE_PKT_LEN,
		     xmouse_irq_out, xmouse, ep_irq_out->bInterval);
    xmouse->irq_out->transfer_dma = xmouse->odata_dma;
    xmouse->irq_out->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

    return 0;

 fail2:	usb_free_coherent(xmouse->udev, XMOUSE_PKT_LEN, xmouse->odata, xmouse->odata_dma);
 fail1:	return error;
}

static void xmouse_deinit_output(struct usb_xmouse *xmouse) {
    usb_free_urb(xmouse->irq_out);
    usb_free_coherent(xmouse->udev, XMOUSE_PKT_LEN, xmouse->odata, xmouse->odata_dma);
}

static void xmouse_stop_output(struct usb_xmouse *xmouse){
    usb_kill_urb(xmouse->irq_out);
}
static int xmouse_init_ff(struct usb_xmouse *xmouse) { return 0; }

//----------------------open and close------------------------
// called when a process tries to open the device file
static int xmouse_open(struct input_dev *dev){
    struct usb_xmouse *xmouse = input_get_drvdata(dev);

    /* USB request block (urb) submission */
    xmouse->irq_in->dev = xmouse->udev;
    if (usb_submit_urb(xmouse->irq_in, GFP_KERNEL))
	return -EIO; // EIO = 5 (I/O errors)
	// so it can't be opened twice
    return 0;
}

static void xmouse_close(struct input_dev *dev){
    struct usb_xmouse *xmouse = input_get_drvdata(dev);
    xmouse_stop_output(xmouse);
}

// ---------------------led controls---------------------------
struct xmouse_led {
 	char name[16];
 	struct led_classdev led_cdev;
 	struct usb_xmouse *xmouse;
};
static void xmouse_send_led_command(struct usb_xmouse *xmouse, int command) {
    if (command >= 0 && command < 14) {
	mutex_lock(&xmouse->odata_mutex);
	xmouse->odata[0] = 0x01;
	xmouse->odata[1] = 0x03;
	xmouse->odata[2] = command;
	xmouse->irq_out->transfer_buffer_length = 3;
	usb_submit_urb(xmouse->irq_out, GFP_KERNEL);
	mutex_unlock(&xmouse->odata_mutex);
    }
}
static void xmouse_led_set(struct led_classdev *led_cdev, enum led_brightness value) {
    struct xmouse_led *xmouse_led = container_of(led_cdev, struct xmouse_led, led_cdev);

    xmouse_send_led_command(xmouse_led->xmouse, value);
}
static int xmouse_led_probe(struct usb_xmouse *xmouse) {
	static atomic_t led_seq	= ATOMIC_INIT(0);
	long led_no;
	struct xmouse_led *led;
	struct led_classdev *led_cdev;
	int error;

	xmouse->led = led = kzalloc(sizeof(struct xmouse_led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led_no = (long)atomic_inc_return(&led_seq) - 1;

	snprintf(led->name, sizeof(led->name), "xmouse%ld", led_no);
	led->xmouse = xmouse;

	led_cdev = &led->led_cdev;
	led_cdev->name = led->name;
	led_cdev->brightness_set = xmouse_led_set;

	error = led_classdev_register(&xmouse->udev->dev, led_cdev);
	if (error) {
		kfree(led);
		xmouse->led = NULL;
		return error;
	}

	/*
	 * Light up in a rotating pattern
	 */
	xmouse_send_led_command(xmouse, 10);

	return 0;
}

static void xmouse_led_disconnect(struct usb_xmouse *xmouse) {
	struct xmouse_led *xmouse_led = xmouse->led;

	if (xmouse_led) {
		led_classdev_unregister(&xmouse_led->led_cdev);
		kfree(xmouse_led);
	}
}

//---------------probe and d/c-------------------------------
static int xmouse_init(struct usb_interface *intf, const struct usb_device_id *id){
    struct usb_device *udev = interface_to_usbdev(intf);
    struct usb_xmouse *xmouse;
    struct input_dev *input_dev;
    struct usb_endpoint_descriptor *ep_irq_in;
    int i, error;
    printk(KERN_ALERT "xmouse loaded");

    /* kzalloc(size, memory_type) zeroes the memory before returning a pointer */
    xmouse = kzalloc(sizeof(struct usb_xmouse), GFP_KERNEL);
    input_dev = input_allocate_device();
    if (!xmouse || !input_dev) { // are either non-zero then error
	error = -ENOMEM;
	goto fail1;
    }

    xmouse->idata = usb_alloc_coherent(udev, XMOUSE_PKT_LEN, GFP_KERNEL, &xmouse->idata_dma);
    if (!xmouse->idata) {
	error = -ENOMEM;
	goto fail1;
    }

    xmouse->irq_in = usb_alloc_urb(0, GFP_KERNEL); 
    if (!xmouse->irq_in) {
	error = -ENOMEM;
	goto fail2;
    }
    
    xmouse->udev = udev;
    xmouse->intf = intf;
    xmouse->dev  = input_dev;
    usb_make_path(udev, xmouse->phys, sizeof(xmouse->phys)); // returns a device path in the usb tree
    strlcat(xmouse->phys, "/input0", sizeof(xmouse->phys)); // copies string to the path we just created
    // why??

    input_dev->name       = "Xbox 360 mouse";
    input_dev->phys       = xmouse->phys;
    usb_to_input_id(udev, &input_dev->id);
    input_dev->dev.parent = &intf->dev;
    input_set_drvdata(input_dev, xmouse);
    input_dev->open       = xmouse_open;
    input_dev->close      = xmouse_close;

    /* specify types of input (abs and key) */
    input_dev->evbit[0]  = BIT_MASK(EV_KEY);
    input_dev->evbit[0] |= BIT_MASK(EV_ABS);
    
    /* set left stick */
    __set_bit(ABS_X, input_dev-> absbit);
    input_set_abs_params(input_dev, ABS_X, -32768, 32767, 1, 16);
    __set_bit(ABS_Y, input_dev-> absbit);
    input_set_abs_params(input_dev, ABS_Y, -32768, 32767, 1, 16);

    /* set buttons (a, b) */
    __set_bit(BTN_LEFT, input_dev->keybit);
    __set_bit(BTN_RIGHT, input_dev->keybit);

    error = xmouse_init_output(intf, xmouse);
    if (error)
	goto fail3;

    error = xmouse_init_ff(xmouse);
    if (error)
	goto fail4;

    error = xmouse_led_probe(xmouse);
    if (error)
	goto fail5;

    ep_irq_in = &intf->cur_altsetting->endpoint[0].desc;
    usb_fill_int_urb(xmouse->irq_in, udev,
		     usb_rcvintpipe(udev, ep_irq_in->bEndpointAddress),
		     xmouse->idata, XMOUSE_PKT_LEN, xmouse_irq_in,
		     xmouse, ep_irq_in->bInterval);
    xmouse->irq_in->transfer_dma = xmouse->idata_dma;
    xmouse->irq_in->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

    error = input_register_device(xmouse->dev);
    if (error)
	goto fail6;

    usb_set_intfdata(intf, xmouse);

    return 0; // if return value is non-zero then module_init failed
    // fail structure for undoing if something fails
    // e.g: if we get far enough to try to open LED and it fails, then undo that and undo everything we did before it
 fail6:	xmouse_led_disconnect(xmouse);
 fail5:	if (input_dev)
		input_ff_destroy(input_dev);
 fail4:	xmouse_deinit_output(xmouse);
 fail3:	usb_free_urb(xmouse->irq_in);
 fail2:	usb_free_coherent(udev, XMOUSE_PKT_LEN, xmouse->idata, xmouse->idata_dma);
 fail1:	input_free_device(input_dev);
	kfree(xmouse);
	return error;
}
static void xmouse_disconnect(struct usb_interface *intf){
    struct usb_xmouse  *xmouse = usb_get_intfdata(intf);

    xmouse_led_disconnect(xmouse);
    input_unregister_device(xmouse->dev);
    xmouse_deinit_output(xmouse);

    usb_free_urb(xmouse->irq_in);
    usb_free_coherent(xmouse->udev, XMOUSE_PKT_LEN, xmouse->idata, xmouse->idata_dma);

    kfree(xmouse);

    printk(KERN_ALERT "xmouse unloaded");

    usb_set_intfdata(intf, NULL);

}

#define VEND 0x045e
#define PROD 0x028e
static struct usb_device_id xmouse_table[] = {
    { USB_DEVICE_INTERFACE_PROTOCOL(VEND, PROD, 1) },
    { },
};
MODULE_DEVICE_TABLE(usb, xmouse_table);

static struct usb_driver xmouse_driver = {
    .name       = "xmouse",
    .probe      = xmouse_init,
    .disconnect = xmouse_disconnect,
    .id_table   = xmouse_table,
};

module_usb_driver(xmouse_driver);

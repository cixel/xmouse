## xmouse
This is the code for a Linux Xbox 360 USB controller driver that allows it to function as a mouse. This only works for the Microsoft controllers and won't react to controllers with a different vendor ID. This is based off of xpad.

Building this requires that you have a configured and built kernel tree on your system. Once you do, running make should generate the kernel modules when can then be loaded into the kernel with the command "sudo insmod ./xmouse.ko".

This was written as a term project for an operating systems class.

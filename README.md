# realtime-tmp

This project contains helps interface with a kernel driver to communicate to motor 
controllers over USB and an API to help communicate with the motor 
controllers. The kernel driver is available at github at unhuman-io/usb_rt_driver. 
The API will install a shared library motor_manager.so and include files: motor.h 
and motor_manager.h. An example is installed in `share/realtime-tmp/example`. A 
very minimal python interface that is compatible with the NVidia Jetbot demo is 
installed in `share/realtime-tmp/python`.

`motor_util` is a command line utility that is also installed. It uses 
the motor_manager library and enumerates any connected devices. 
Example:
```console
$ motor_util
motor_util version: 0.1.4
1 connected motor
         Dev      Name  Serial number                             Version           Path
----------------------------------------------------------------------------------------
   /dev/mtr1        J1           0001          0.2.1 Sep  4 2019 13:48:21    1-4.2.1.3.1
```
`motor_util` also allows for simple reading and writing to the motors with the set and read commands. Tab completion and -h help is supported. Note that when setting you always need to set the mode or else it 
will default to `open`. Some examples are: 
```console
$ motor_util set --mode current --current 1
$ motor_util read
```
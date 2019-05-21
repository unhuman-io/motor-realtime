realtime-tmp

This project contains a kernel driver to communicate to motor 
controllers over USB and an API to help communicate with the motor 
controllers. The project will install the kernel driver in 
/lib/modules/${uname -r}. The API will install a shared library 
motor_manager.so and include files: motor.h and motor_manager.h. 
An example is installed in share/realtime-tmp/example.
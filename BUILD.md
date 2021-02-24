Dependencies
- cmake
- libudev-dev
- kernel headers

For example:
```
$ sudo apt install libudev-dev cmake linux-headers-$(uname -r)
```

CMake build process. For example from the root directory
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

make package generates a debian package which should be installed in order
to install the usb driver. 

An example program "deadline" is built in the examples folder. This is a two 
thread realtime program example that runs a read/modify/write control loop at 
a somewhat fixed frequency. It uses the accessory controller.{h,cpp} files to 
implement a basic position control. Modifying the controller and deadline files 
is a recommended method for testing the system. A typical example is:
- Modify deadline.cpp control code to implement a trajectory
  - For example: `controller_.set_position(sin(controller_.set_position(sin(std::chrono::duration_cast<std::chrono::nanoseconds>(data_.time_start - start_time_).count()/1e9));`
- Build:
  - `make`
- Run:
  - `./example/deadline`
- Examine results:
  - `head data.csv`
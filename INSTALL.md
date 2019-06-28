It's recommended to make and install the dpkg
```
$ make package
$ sudo dpkg -i realtime-*.deb
```

udev rules are helpful to connect to usb. They are installed
with the package or do:
```
$ sudo cp share/99-realtime-tmp.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
```

Also depmod is run by the package install, or do:
```
$ sudo depmod
```

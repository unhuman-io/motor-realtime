``
sudo apt-get install qemu binfmt-support qemu-user-static
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
docker run --platform=linux/arm64/v8 -it -v .:/motor-realtime arm64v8/ubuntu
cd motor-realtime/build_a
apt update
apt install -y cmake build-essential libudev-dev git python3-dev
cmake ..
make
```

later can try
```
docker start `docker ps -q -l`
docker attach `docker ps -q -l`
```

```
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=artifacts -DINSTALL_COMPLETION=off .
make -j2 VERBOSE=1
```

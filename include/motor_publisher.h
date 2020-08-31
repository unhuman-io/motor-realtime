#pragma once
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>

class MotorPublisher {
 public:
    MotorPublisher() {
        fd_ = shm_open(shm_, O_RDWR  | O_CREAT, 0666);
        ftruncate(fd_, 1000);
        memptr_ = mmap(NULL,       /* let system pick where to put segment */
                        1000,   /* how many bytes */
                        PROT_READ | PROT_WRITE, /* access protections */
                        MAP_SHARED, /* mapping visible to other processes */
                        fd_,         /* file descriptor */
                        0);
        std::memset(memptr_, 0, 1000);
    }
    ~MotorPublisher() {
        munmap(memptr_, 1000);
        close(fd_);
        shm_unlink(shm_);
    }
    void publish(std::string str) {
        std::strcpy((char *) memptr_, str.c_str());
    }
 private:
    int fd_;
    char shm_[100] = "motor_data";
    void * memptr_;
};

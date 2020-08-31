#pragma once
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>

class MotorSubscriber {
 public:
    MotorSubscriber() {
        fd_ = shm_open(shm_, O_RDONLY, 0666);
        memptr_ = mmap(NULL,       /* let system pick where to put segment */
                        1000,   /* how many bytes */
                        PROT_READ, /* access protections */
                        MAP_SHARED, /* mapping visible to other processes */
                        fd_,         /* file descriptor */
                        0);
    }
    ~MotorSubscriber() {
        munmap(memptr_, 1000);
        close(fd_);
        shm_unlink(shm_);
    }
    std::string read() {
        char s[1000];
        std::strncpy(s, (char *) memptr_, 1000);
        std::string str = s;
        return str;
    }
 private:
    int fd_;
    char shm_[100] = "motor_data";
    void * memptr_;
};

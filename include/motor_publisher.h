#pragma once
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>
#include "cstack.h"

template <class T>
class MotorPublisher {
 public:
    MotorPublisher(std::string shm_name = "motor_data") : shm_name_(shm_name) {
        fd_ = shm_open(shm_name_.c_str(), O_RDWR  | O_CREAT, 0666);
        ftruncate(fd_, sizeof(*data_));
        memptr_ = mmap(nullptr,       /* let system pick where to put segment */
                        sizeof(*data_),   /* how many bytes */
                        PROT_READ | PROT_WRITE, /* access protections */
                        MAP_SHARED, /* mapping visible to other processes */
                        fd_,         /* file descriptor */
                        0);
        std::memset(memptr_, 0, 1000);
        data_ = reinterpret_cast<CStack<T> *>(memptr_);
    }
    ~MotorPublisher() {
        munmap(memptr_, sizeof(*data_));
        close(fd_);
        shm_unlink(shm_name_.c_str());
    }
    void publish(T data) {
        data_->push(data);
        //std::strcpy((char *) memptr_, str.c_str());
    }
 private:
    int fd_;
    std::string shm_name_;
    void * memptr_;
    CStack<T> *data_;
};

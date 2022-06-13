#pragma once
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>
#include <semaphore.h>
#include <cstack.h>

template <class T>
class MotorSubscriber {
 public:
    MotorSubscriber(std::string shm_name = "motor_data") : shm_name_(shm_name) {
        open();
    }
    ~MotorSubscriber() {
        munmap(memptr_, sizeof(*data_));
        close(fd_);
    }
    T read() {
        T data = {};
        if (fd_ > 0) {
            data = data_->top();
        } else {
            open();
        }
        return data;
    }
 private:
    void open() {
        fd_ = shm_open(shm_name_.c_str(), O_RDONLY, 0666);
        if (fd_ > 0) {
            memptr_ = mmap(nullptr,       /* let system pick where to put segment */
                        sizeof(*data_),   /* how many bytes */
                        PROT_READ, /* access protections */
                        MAP_SHARED, /* mapping visible to other processes */
                        fd_,         /* file descriptor */
                        0);
            data_ = reinterpret_cast<CStack<T> *>(memptr_);
        }
    }
    int fd_;
    std::string shm_name_;
    void * memptr_;
    CStack<T> * data_;
};

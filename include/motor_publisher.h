#pragma once
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>
#include "cstack.h"
#include <chrono>

namespace obot {

template <class T>
class MotorPublisher {
 public:
    struct Header {
        uint64_t timestamp;
        uint64_t seq_num;
    };
    struct Data {
        Header header;
        T data;
    };
    MotorPublisher(std::string shm_name = "motor_data") : shm_name_(shm_name) {
        fd_ = shm_open(shm_name_.c_str(), O_RDWR  | O_CREAT, 0666);
        ftruncate(fd_, sizeof(*data_));
        memptr_ = mmap(nullptr,       /* let system pick where to put segment */
                        sizeof(*data_),   /* how many bytes */
                        PROT_READ | PROT_WRITE, /* access protections */
                        MAP_SHARED, /* mapping visible to other processes */
                        fd_,         /* file descriptor */
                        0);
        std::memset(memptr_, 0, sizeof(*data_));
        data_ = reinterpret_cast<CStack<Data> *>(memptr_);
    }
    ~MotorPublisher() {
        data_->close();
        munmap(memptr_, sizeof(*data_));
        close(fd_);
        shm_unlink(shm_name_.c_str());
    }
    void publish(T data) {
        auto now = std::chrono::steady_clock::now();
        auto since_epoch = now.time_since_epoch();
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch);
        data_struct_.header.timestamp = nanoseconds.count();
        data_struct_.header.seq_num++;
        data_struct_.data = data;
        data_->push(data_struct_);
    }
 private:
    int fd_;
    Data data_struct_ = {};
    std::string shm_name_;
    void * memptr_;
    CStack<Data> *data_;
};

}  // namespace obot

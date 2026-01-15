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
#include "motor_publisher.h"

namespace obot {

template <class T>
class MotorSubscriber {
 public:
    using Data = typename MotorPublisher<T>::Data;
    MotorSubscriber(std::string shm_name = "motor_data") : shm_name_(shm_name) {
        open();
    }
    ~MotorSubscriber() {
        munmap(memptr_, sizeof(*data_));
        close(fd_);
        shm_unlink(shm_name_.c_str());
    }
    T read() {
        Data data = {};
        update_stats(data.header);
        if (fd_ > 0) {
            data = data_->top();
        } else {
            open();
        }
        return data.data;
    }
    uint64_t get_dt() const {
        return dt_;
    }
 private:
    void update_stats(typename MotorPublisher<T>::Header & header) {
        if (header.seq_num > last_seq_num_) {
            last_seq_num_ = header.seq_num;
            last_timestamp_ = header.timestamp;
            dt_ = (header.timestamp - last_timestamp_)/(header.seq_num - last_seq_num_);
            last_timestamp_ = header.timestamp;
            last_seq_num_ = header.seq_num;
        }  
    }
    void open() {
        fd_ = shm_open(shm_name_.c_str(), O_RDONLY, 0666);
        if (fd_ > 0) {
            memptr_ = mmap(nullptr,       /* let system pick where to put segment */
                        sizeof(*data_),   /* how many bytes */
                        PROT_READ, /* access protections */
                        MAP_SHARED, /* mapping visible to other processes */
                        fd_,         /* file descriptor */
                        0);
            data_ = reinterpret_cast<CStack<Data> *>(memptr_);
        }
    }
    int fd_;
    std::string shm_name_;
    void * memptr_;
    CStack<Data> * data_;
    uint64_t last_timestamp_ = 0;
    uint64_t last_seq_num_ = 0;
    uint64_t dt_ = 0;  
};

}  // namespace obot

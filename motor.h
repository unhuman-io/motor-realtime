#ifndef MOTOR_H
#define MOTOR_H

#include <fcntl.h>

class Motor {
 public:
    ~Motor() { ::close(fid_); }
    int open(const char *path) { fid_ = ::open(path, O_RDWR); fid_flags_ = fcntl(fid_, F_GETFL); };
    ssize_t read(void *buf, size_t count) { return ::read(fid_, buf, count); };
    ssize_t write(const void *buf, size_t count) { return ::write(fid_, buf, count); };
    ssize_t aread(void *buf, size_t count) { int fcntl_error = fcntl(fid_, F_SETFL, fid_flags_ | O_NONBLOCK);
			ssize_t read_error = read(buf, count); 
            fcntl_error = fcntl(fid_, F_SETFL, fid_flags_);
            if (read_error != -1) {
                std::cout << "Nonzero aread" << std::endl;
            } else {
                if (errno != EAGAIN) {
                    std::cout << "Aread error " << errno << std::endl;
                }
            }
            return read_error; }
 private:
    int fid_ = 0;
    int fid_flags_;
};

#endif

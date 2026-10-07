#include <liburing.h>
#include "motor_manager.h"

namespace obot {

std::vector<Status> &MotorManager::read_uring() {
    io_uring ring;
    io_uring_queue_init(motors_.size(), &ring, 0);
    
    for (int i=0; i < motors_.size(); i++) {
        io_uring_sqe *sqe = io_uring_get_sqe(&ring);
        io_uring_sqe_set_data64(sqe, i);
        io_uring_prep_read(sqe, motors_[i]->fd(), motors_[i]->read_buffer(), motors_[i]->read_buffer_size(), 0);
    }
    if (int retval = io_uring_submit(&ring); retval < 0) {
        throw RuntimeErrnoException("io_uring_submit");
    } else {
        std::cout << "io_uring_submit: " << retval << std::endl;
    }
    io_uring_cqe *cqe;
    if (int retval = io_uring_wait_cqes(&ring, &cqe, motors_.size(), 0, 0); retval < 0) {
        throw RuntimeErrnoException("io_uring_wait_cqe");
    } else {
        std::cout << "io_uring_wait_cqe: " << retval << std::endl;
        unsigned head;
        unsigned processed = 0;
        io_uring_for_each_cqe(&ring, head, cqe) {
            int i = io_uring_cqe_get_data64(cqe);
            Motor* motor = motors_[i].get();
            
            if (cqe->res == motor->read_buffer_size()) {
                // Success! Statuses_[idx] is now updated
                std::cout << "Motor " << i << " read success." << std::endl;
                motor->process_read_buffer();
                statuses_[i] = *motor->status();
            } else {
                std::cerr << "Motor " << " failed with res: " << cqe->res << std::endl;
            }
            processed++;
        }
        io_uring_cq_advance(&ring, processed);
    }
    io_uring_queue_exit(&ring);
    return statuses_;
}

}; // namespace obot

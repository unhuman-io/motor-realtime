#pragma once

#include <atomic>
#include <stdexcept>

namespace obot {

// A circular stack. If data is written by one thread and read by one other thread then data is read from the top without worrying about thread safety.
template <class T, int size=100>
class CStack {
 public:
    void push(T const &t) {
		int pos_old = pos_.load(std::memory_order_acquire);
		if (pos_old == -1) {
			throw std::runtime_error("CStack is closed");
		}
		int future_pos = pos_old + 1;
		if (future_pos >= size) {
			future_pos = 0;
		}
		data_[future_pos] = t;
		pos_.store(future_pos, std::memory_order_release);
	}
	T top() const { // return a copy of the data
		int pos = pos_.load(std::memory_order_acquire);
		if (pos == -1) {
			throw std::runtime_error("CStack is closed");
		}
		return data_[pos];
	}
	void close() {
		pos_.store(-1, std::memory_order_release);
	}
 private:
	T data_[size] = {};
	std::atomic<int> pos_ = {0};
};

}  // namespace obot

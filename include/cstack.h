#pragma once

#include <atomic>

namespace obot {

// A circular stack. If data is written by one thread and read by one other thread then data is read from the top without worrying about thread safety.
template <class T, int size=100>
class CStack {
 public:
    void push(T const &t) {
		int future_pos = pos_.load(std::memory_order_acquire) + 1;
		if (future_pos >= size) {
			future_pos = 0;
		}
		data_[future_pos] = t;
		pos_.store(future_pos, std::memory_order_release);
	}
	T top() const { // return a copy of the data
		return data_[pos_.load(std::memory_order_acquire)];
	}
 private:
	T data_[size] = {};
	std::atomic<int> pos_ = {0};
};

}  // namespace obot

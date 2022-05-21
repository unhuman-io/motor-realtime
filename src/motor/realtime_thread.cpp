#include "realtime_thread.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <sys/syscall.h>
#include <sys/mman.h>

#include <chrono>
#include <thread>
#include <iostream>

#define gettid() syscall(__NR_gettid)

#define SCHED_DEADLINE	6

/* XXX use the proper syscall numbers */
#ifdef __x86_64__
#define __NR_sched_setattr		314
#define __NR_sched_getattr		315
#endif

#ifdef __i386__
#define __NR_sched_setattr		351
#define __NR_sched_getattr		352
#endif

#ifdef __arm__
#define __NR_sched_setattr		380
#define __NR_sched_getattr		381
#endif

static volatile int done;

struct sched_attr {
	__u32 size;

	__u32 sched_policy;
	__u64 sched_flags;

	/* SCHED_NORMAL, SCHED_BATCH */
	__s32 sched_nice;

	/* SCHED_FIFO, SCHED_RR */
	__u32 sched_priority;

	/* SCHED_DEADLINE (nsec) */
	__u64 sched_runtime;
	__u64 sched_deadline;
	__u64 sched_period;
};

int sched_setattr(pid_t pid,
		const struct sched_attr *attr,
		unsigned int flags)
{
return syscall(__NR_sched_setattr, pid, attr, flags);
}

int sched_getattr(pid_t pid,
		struct sched_attr *attr,
		unsigned int size,
		unsigned int flags)
{
return syscall(__NR_sched_getattr, pid, attr, size, flags);
}


void RealtimeThread::run() { 
	done_ = false;
	thread_ = new std::thread([=]{run_deadline();}); 
}

void RealtimeThread::done() {
	done_ = true; 
	if (exit_.get_future().wait_for(std::chrono::nanoseconds(period_ns_)*2) == std::future_status::timeout) {
		std::cerr << "Difficulty stopping realtime thread" << std::endl;
	} else {
		thread_->join();
		//printf("Realtime thread joined\n");
	}
	delete thread_;
}

void RealtimeThread::jitter_error(int32_t time_jitter_ns) {
	jitter_error_++;
}
void RealtimeThread::run_deadline()
{
	//printf("realtime thread started period_ns = %d, [%ld]\n", period_ns_, gettid());
	exit_ = std::promise<void>();

	struct sched_attr attr;
	attr.size = sizeof(attr);
	attr.sched_flags = 0;
	attr.sched_nice = 0;
	attr.sched_priority = 0;

	attr.sched_policy = SCHED_DEADLINE;
	attr.sched_runtime =  period_ns_*9.0/10;
	attr.sched_deadline = period_ns_*9.0/10;
	attr.sched_period =  period_ns_;

	bool deadline_permissions = true;
	unsigned int flags = 0;
	auto ret = sched_setattr(0, &attr, flags);
	if (ret < 0) {
		if (debug_) {
			perror("Error with sched_setattr");
		}
		deadline_permissions = false;
		if (debug_) {
			printf("Running std::this_thread::sleep_until mode\n");
		}
	} else {
		if (debug_) {
			printf("Running deadline scheduler\n");
		}
	}

	ret = mlockall(MCL_CURRENT | MCL_FUTURE);
	if (ret < 0) {
		if (debug_) {
			perror("Error locking memory");
		}
	}

	auto next_time = std::chrono::steady_clock::now();
	start_time_ = next_time;
	auto last_loop_start_time = start_time_ - std::chrono::nanoseconds(period_ns_);
	while (!done_) {
		auto loop_start_time = std::chrono::steady_clock::now();
		auto period = loop_start_time - last_loop_start_time;
		int32_t time_jitter = std::chrono::duration_cast<std::chrono::nanoseconds>(period).count() - period_ns_;
		
		
		if (abs(time_jitter) > max_jitter_ns_) {
			jitter_error(time_jitter);
		}

		update();

		last_loop_start_time = loop_start_time;
		next_time += std::chrono::nanoseconds(period_ns_);
		if(!deadline_permissions) {
			std::this_thread::sleep_until(next_time);
		} else {
			sched_yield();
		}
	}

	exit_.set_value();
	//printf("realtime thread finish [%ld]\n", gettid());
}

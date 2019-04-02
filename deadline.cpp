#define _GNU_SOURCE
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <sys/syscall.h>

#include <thread>
#include <iostream>
#include <cstring>
#include <chrono>
#include <fcntl.h>
#include <libudev.h>
#include <fcntl.h>
#include <errno.h>

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

struct Data {
	struct D {
		int32_t count;
		int32_t count_received;
		float res[3];
	} buffer;
	struct C {
		int32_t count;
	} command;
	int32_t delay;
	std::chrono::steady_clock::time_point time_start, last_time_start, last_time_end;
};

template <class T>
class CStack {
 public:
    void push(T &t) {
		int future_pos = pos_ + 1;
		if (future_pos >= 100) {
			future_pos = 0;
		}
		data_[future_pos] = t;
		pos_ = future_pos;
	}
	T top() {
		return data_[pos_];
	}
 private:
	T data_[100];
	int pos_ = 0;
};

char * j1_dev_path;
class Task {
 public:
    Task(CStack<Data> &cstack) : cstack_(cstack) {
		// fid_ = fopen("/dev/skel0", "r");
		// fid_out_ = fopen("/dev/skel1", "w");
		std::cout << "open: " << j1_dev_path << std::endl;
		fid_ = open(j1_dev_path, O_RDWR);
		fid_flags_ = fcntl(fid_, F_GETFL);
		//fid_out_ = open("/dev/skel1", O_WRONLY);
		//std::cout << "fidout" << fid_out_;
	}
	~Task() {
		// fclose(fid_);
		// fclose(fid_out_);
		close(fid_);
		//close(fid_out_);
	}
	void run() { done_ = 0;
		start_time_ = std::chrono::steady_clock::now();
		next_time_ = start_time_;
		thread_ = new std::thread([=]{run_deadline();}); }
	void done() { done_ = 1; }
	void join() { thread_->join(); }
 private:
	void run_deadline()
	{
		struct sched_attr attr;
		int32_t x = 0;
		int ret;
		unsigned int flags = 0;

		printf("deadline thread started [%ld]\n", gettid());

		attr.size = sizeof(attr);
		attr.sched_flags = 0;
		attr.sched_nice = 0;
		attr.sched_priority = 0;

		attr.sched_policy = SCHED_DEADLINE;
		attr.sched_runtime =  150 * 1000;
		attr.sched_deadline = 200 * 1000;
		attr.sched_period =  period_ns_;

		int not_root = 0;
		ret = sched_setattr(0, &attr, flags);
		if (ret < 0) {
			perror("sched_setattr");
			not_root = 1;
		}

		while (!done_) {
			x++;
			next_time_ += std::chrono::nanoseconds(period_ns_);
			data_.last_time_start = data_.time_start;
			data_.time_start = std::chrono::steady_clock::now();

			//int read_error = fread(&data_.buffer,1 ,sizeof(data_.buffer), fid_);
			int fcntl_error = fcntl(fid_, F_SETFL, fid_flags_ | O_NONBLOCK);
			int read_error = read(fid_, &data_.buffer, sizeof(data_.buffer)); // expect errno EAGAIN
			if (read_error < 0) {
	//			std::cout << "read error: " << errno << std::endl;
			} else {
				// actually read some data even though async
				// should skip subsequent read
			}
			fcntl_error = fcntl(fid_, F_SETFL, fid_flags_);

			// blocking io to get the data alread set up and wait if not ready yet
			read_error = read(fid_, &data_.buffer, sizeof(data_.buffer));
			if (read_error < 0) {
				std::cout << "read error: " << errno << std::endl;
			} else {
		//		std::cout << "read success: " << read_error << std::endl;
			}

			data_.command.count = x;
			data_.delay = x - data_.buffer.count_received;
			if (data_.delay > 1) {
				std::cout << "Delay > 1: " << data_.delay << std::endl;
			}
			int write_error = write(fid_, &data_.command, 4);
			if (write_error < 0) {
				std::cout << "write error: " << strerror(-write_error) << std::endl;
			}
			//fflush(fid_out_);

			cstack_.push(data_);
			data_.last_time_end = std::chrono::steady_clock::now();

			if(not_root) {
				std::this_thread::sleep_until(next_time_);
			} else {
				sched_yield();
			}
		}

		printf("deadline thread dies [%ld]\n", gettid());
	}
	std::thread *thread_;
	int done_;
	CStack<Data> &cstack_;
	Data data_;
	std::chrono::steady_clock::time_point start_time_, next_time_;
	long period_ns_ =   500 * 1000;
	// FILE *fid_, *fid_out_;
	int fid_, fid_out_;
	int fid_flags_;
};

int udev (void)
{
	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev;
	
	/* Create the udev object */
	udev = udev_new();
	if (!udev) {
		printf("Can't create udev\n");
		exit(1);
	}
	
	/* Create a list of the devices in the 'hidraw' subsystem. */
	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_sysname(enumerate, "skel*");
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);
	/* For each item enumerated, print out its information.
	   udev_list_entry_foreach is a macro which expands to
	   a loop. The loop will be executed for each member in
	   devices, setting dev_list_entry to a list entry
	   which contains the device's path in /sys. */
	udev_list_entry_foreach(dev_list_entry, devices) {
		const char *path;
		
		/* Get the filename of the /sys entry for the device
		   and create a udev_device object (dev) representing it */
		path = udev_list_entry_get_name(dev_list_entry);
		const char * val = udev_list_entry_get_value(dev_list_entry);
		dev = udev_device_new_from_syspath(udev, path);

		/* usb_device_get_devnode() returns the path to the device node
		   itself in /dev. */
		printf("Device Node Path: %s, %s, %s\n", udev_device_get_devnode(dev), path, udev_device_get_sysname(dev));
		printf("  name: %s\n",
		         udev_device_get_sysattr_value(dev, "device/interface"));
		if (strcmp("J1",udev_device_get_sysattr_value(dev, "device/interface")) == 0) {
			std::cout << "strcmp J1 **********" << std::endl;
			const char * devpath = udev_device_get_devnode(dev);
			j1_dev_path = new char[std::strlen(devpath)];
			std::strcpy(j1_dev_path, devpath);
		}
		/* The device pointed to by dev contains information about
		   the hidraw device. In order to get information about the
		   USB device, get the parent device with the
		   subsystem/devtype pair of "usb"/"usb_device". This will
		   be several levels up the tree, but the function will find
		   it.*/
		dev = udev_device_get_parent_with_subsystem_devtype(
		       dev,
		       "usb",
		       "usb_device");
		if (!dev) {
			printf("Unable to find parent usb device.");
			exit(1);
		}
	
		/* From here, we can call get_sysattr_value() for each file
		   in the device's /sys entry. The strings passed into these
		   functions (idProduct, idVendor, serial, etc.) correspond
		   directly to the files in the directory which represents
		   the USB device. Note that USB strings are Unicode, UCS2
		   encoded, but the strings returned from
		   udev_device_get_sysattr_value() are UTF-8 encoded. */
		printf("  VID/PID: %s %s\n",
		        udev_device_get_sysattr_value(dev,"idVendor"),
		        udev_device_get_sysattr_value(dev, "idProduct"));
		printf("  %s\n  %s\n",
		        udev_device_get_sysattr_value(dev,"manufacturer"),
		        udev_device_get_sysattr_value(dev,"product"));
		printf("  serial: %s\n",
		         udev_device_get_sysattr_value(dev, "serial"));
		printf("  name: %s\n",
		         udev_device_get_sysattr_value(dev, "name"));
		udev_device_unref(dev);
	}
	/* Free the enumerator object */
	udev_enumerate_unref(enumerate);

	udev_unref(udev);

	return 0;       
}

int main (int argc, char **argv)
{
	udev();
	printf("main thread [%ld]\n", gettid());
	CStack<Data> cstack;
	Task task(cstack);
	task.run();
	std::chrono::steady_clock::time_point system_start = std::chrono::steady_clock::now();

	for(int i=0; i<10; i++) {
		Data data = cstack.top();
		auto last_exec = std::chrono::duration_cast<std::chrono::nanoseconds>(data.last_time_end - data.last_time_start).count();
		auto last_period =  std::chrono::duration_cast<std::chrono::nanoseconds>(data.time_start - data.last_time_start).count();
		auto start = std::chrono::duration_cast<std::chrono::nanoseconds>(data.time_start - system_start).count();
		std::cout << "last_period: " << last_period << " last_exec: " << last_exec \
				<< " count_received: " << data.buffer.count_received << "current_count: " << data.command.count << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	task.done();
	task.join();


	printf("main dies [%ld]\n", gettid());
	return 0;
}

#include "CLI11.hpp"
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sys/ioctl.h>
#include <cstdint>
#include "motor_manager.h"

#define MON_IOC_MAGIC 0x92

#define MON_IOCQ_URB_LEN _IO(MON_IOC_MAGIC, 1)
/* #2 used to be MON_IOCX_URB, removed before it got into Linus tree */
#define MON_IOCG_STATS _IOR(MON_IOC_MAGIC, 3, struct mon_bin_stats)
#define MON_IOCT_RING_SIZE _IO(MON_IOC_MAGIC, 4)
#define MON_IOCQ_RING_SIZE _IO(MON_IOC_MAGIC, 5)
#define MON_IOCX_GET   _IOW(MON_IOC_MAGIC, 6, struct mon_bin_get)
#define MON_IOCX_MFETCH _IOWR(MON_IOC_MAGIC, 7, struct mon_bin_mfetch)
#define MON_IOCH_MFLUSH _IO(MON_IOC_MAGIC, 8)
/* #9 was MON_IOCT_SETAPI */
#define MON_IOCX_GETX   _IOW(MON_IOC_MAGIC, 10, struct mon_bin_get)

#define SETUP_LEN  8

using obot::Status;
using obot::Command;

struct mon_bin_hdr {
      uint64_t id;                 /*  0: URB ID - from submission to callback */
      unsigned char type;     /*  8: Same as text; extensible. */
      unsigned char xfer_type; /*    ISO (0), Intr, Control, Bulk (3) */
      unsigned char epnum;    /*     Endpoint number and transfer direction */
      unsigned char devnum;   /*     Device address */
      uint16_t busnum;             /* 12: Bus number */
      char flag_setup;        /* 14: Same as text */
      char flag_data;         /* 15: Same as text; Binary zero is OK. */
      int64_t ts_sec;             /* 16: gettimeofday */
      int32_t ts_usec;            /* 24: gettimeofday */
      int status;             /* 28: */
      unsigned int length;    /* 32: Length of data (submitted or actual) */
      unsigned int len_cap;   /* 36: Delivered length */
      union {                 /* 40: */
              unsigned char setup[SETUP_LEN]; /* Only for Control S-type */
              struct iso_rec {                /* Only for ISO */
                      int error_count;
                      int numdesc;
              } iso;
      } s;
      int interval;           /* 48: Only for Interrupt and ISO */
      int start_frame;        /* 52: For ISO */
      unsigned int xfer_flags; /* 56: copy of URB's transfer_flags */
      unsigned int ndesc;     /* 60: Actual number of ISO descriptors */
};

struct mon_bin_get {
    struct mon_bin_hdr *hdr; /* Can be 48 bytes or 64. */
    void *data;
    size_t alloc;       /* Length of data (can be zero) */
};

#define ERRNO_STR std::to_string(errno) + ": " + strerror(errno)

#define URB_DIR_IN		0x0200	/* Transfer from device to host */
#define URB_DIR_OUT		0
#define URB_DIR_MASK	URB_DIR_IN

bool is_in(unsigned int xfer_flags) { return (xfer_flags & URB_DIR_MASK) == URB_DIR_IN;}

std::string escape_string(std::string s) {
    std::string out;
    for (auto &c : s) {
        switch (c) {
            case '\b':
                out += "\\b";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += c;
                break;
        }
    }
    return out;
}

int main(int argc, char** argv) {
    CLI::App app{"Utility for parsing usbmon data for motors"};
    std::vector<uint8_t> device_numbers = {};
    std::string usbmon_devname = "/dev/usbmon1";
    app.add_option("-m", usbmon_devname, "Usbmon devname", true)->type_name("DEVNAME");
    app.add_option("device_numbers,-d", device_numbers, "Parse for usb device number(s)")->type_name("DEVICE_NUMBER")->expected(1,-1)->required();
    CLI11_PARSE(app, argc, argv);

    int fd = open(usbmon_devname.c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << "Error opening device " << usbmon_devname << " " << std::to_string(errno) + ": " + strerror(errno) << std::endl;
        return 1;
    }

    while(1) {
        uint8_t data[65];
        mon_bin_hdr hdr;
        mon_bin_get bin_get = {.hdr = &hdr, .data = data, .alloc = 64};

        int retval = ioctl(fd, MON_IOCX_GETX, &bin_get);
        if (retval < 0) {
            std::cerr << "ioctl error " <<  ERRNO_STR  << std::endl;
        }
        if (std::find(device_numbers.begin(), device_numbers.end(), hdr.devnum) != device_numbers.end()) {
            std::cout << std::setw(3) << (int) hdr.devnum << " " << hdr.type << (is_in(hdr.xfer_flags) ? 'i' : 'o') << ", " << std::setw(6) << hdr.ts_usec << ", ";
            std::cout << std::hex << std::setw(2) << (int) hdr.epnum << std::dec << ", ";
            if ((hdr.epnum & 0x7F) == 1) {
                // text api
                data[hdr.len_cap] = 0;
                std::cout << escape_string((char*) data);
            } else {
                if (is_in(hdr.xfer_flags)) {
                    if (hdr.type == 'C') {
                        std::vector<Status> status = {*(Status *) data};
                        std::cout << status;
                    }
                } else {
                    std::vector<Command> command = {*(Command *) data};
                    std::cout << command;
                }
            }
            std::cout << std::endl;
        }
    }

    close(fd);

    return 0;
}
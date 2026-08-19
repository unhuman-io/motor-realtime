#include "motor_eth_l2.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <net/ethernet.h>
#ifdef __linux__
#include <linux/if_packet.h>
#include <linux/filter.h>
#include <net/if_arp.h>
#elif defined(__APPLE__)
#include <net/bpf.h>
#include <net/if_dl.h>
#include <net/if_types.h>
#include <fcntl.h>
#endif

#include <ifaddrs.h>
#include <array>
#include <sstream>
#include <iomanip>

#include "poll.h"

namespace obot {

class RuntimeHexDumpException : public RuntimeException {
  public:
    RuntimeHexDumpException(const std::string& msg, uint8_t *data_ptr, int length) :
        RuntimeException(
            msg + "\n" + hex_dump(data_ptr, length)
        ) {}
    static std::string hex_dump(uint8_t *data_ptr, int length) {
        std::stringstream stream;
        stream << "    hex dump length: " << length;
        stream << std::hex << std::setfill('0') << std::setw(2);
        for (int i = 0; i < length; i++ ) {
            if (i % 16 == 0) {
                stream << "\n    ";
            }
            stream << std::setw(2) << (int) data_ptr[i] << " ";
            
        }
        return stream.str();
    }
};

enum class L2MessageType
{
    OBOT_CMD = 0x01,
    OBOT_CMD_STATUS = 0x02,
    OBOT_STATUS = 0x03,
    OBOT_ASCII_CMD = 0x04,
    OBOT_ASCII_RESPONSE = 0x05,
    OBOT_ENUM = 0xF,
};

constexpr int MAX_ETH_L2_PAYLOAD_SIZE = 3000;
constexpr int MAX_PAYLOAD_LENGTH = 64;
using mac_t = std::array<uint8_t, 6>;
struct L2Frame {
    mac_t dst_mac = {};
    mac_t src_mac = {};
    uint8_t ethertype[2] = {0x88, 0xB5};
    uint8_t reserved[6] = {};
    uint8_t payload[MAX_ETH_L2_PAYLOAD_SIZE] = {};
};
constexpr int L2_HEADER_SIZE = sizeof(L2Frame) - sizeof(L2Frame::payload);
static_assert(L2_HEADER_SIZE == 20);

struct Payload {
    uint16_t topic_id;
    uint16_t length;
    uint8_t* data;
};
constexpr int PAYLOAD_HEADER_SIZE = sizeof(Payload::topic_id) + sizeof(Payload::length);
static_assert(PAYLOAD_HEADER_SIZE == 4);

struct TopicId {
    uint16_t node_id:4;
    uint16_t bus_id:3;
    uint16_t type:4;
};

mac_t str2mac(std::string mac_str) {
    mac_t mac;
    if (sscanf(mac_str.c_str(), "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
        &mac[0], &mac[1], &mac[2],
        &mac[3], &mac[4], &mac[5]) == 6) {
    } else {
        throw RuntimeException("Invalid MAC address format (" + mac_str + ")");
    }
    return mac;
}

std::string mac2str(const mac_t &mac) {
    char hex_string[18];
    snprintf(hex_string, sizeof(hex_string), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return hex_string;
}

mac_t get_interface_mac_address(int fd, std::string interface) {
    mac_t mac {};
    if (interface != "any" ) {
#ifdef __linux__
        struct ifreq ifr = {};
        std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
        if (ioctl(fd, SIOCGIFHWADDR, &ifr) == -1) {
            throw RuntimeErrnoException("ioctl SIOCGIFHWADDR failed for " + interface);
        }
        std::memcpy(mac.data(), ifr.ifr_hwaddr.sa_data, 6);
#elif defined(__APPLE__)
        (void)fd;  // macOS reads the link-layer address via getifaddrs(), not the socket
        struct ifaddrs *addrs = nullptr;
        if (getifaddrs(&addrs)) {
            throw RuntimeErrnoException("getifaddrs failed for " + interface);
        }
        bool found = false;
        for (struct ifaddrs *tmp = addrs; tmp != nullptr; tmp = tmp->ifa_next) {
            if (tmp->ifa_addr && tmp->ifa_addr->sa_family == AF_LINK &&
                interface == tmp->ifa_name) {
                struct sockaddr_dl *sdl = (struct sockaddr_dl *)tmp->ifa_addr;
                if (sdl->sdl_alen == 6) {
                    std::memcpy(mac.data(), LLADDR(sdl), 6);
                    found = true;
                    break;
                }
            }
        }
        freeifaddrs(addrs);
        if (!found) {
            throw RuntimeException("Could not get MAC address for " + interface);
        }
#endif
    }
    return mac;
}

std::vector<std::string> get_eth_interfaces() {
    std::vector<std::string> interfaces;
    struct ifaddrs *addrs,*tmp;

    if (getifaddrs(&addrs)) {
        throw RuntimeException("Error getting interfaces: " + std::to_string(errno) + ": " + strerror(errno));
    }
    tmp = addrs;

    while (tmp) {
        //std::cout << "interface: " << tmp->ifa_name << " flags " << tmp->ifa_flags << std::endl;
        // test if interface is ethernet
        if (tmp->ifa_flags & IFF_UP) {
#ifdef __linux__
            struct sockaddr_ll *sll = (struct sockaddr_ll *)tmp->ifa_addr;
            if (sll && sll->sll_hatype == ARPHRD_ETHER) {
                //printf("Interface %s is Ethernet-compatible\n", tmp->ifa_name);
                interfaces.push_back(tmp->ifa_name);
            }
#elif defined(__APPLE__)
            if (tmp->ifa_addr && tmp->ifa_addr->sa_family == AF_LINK) {
                struct sockaddr_dl *sdl = (struct sockaddr_dl *)tmp->ifa_addr;
                if (sdl->sdl_type == IFT_ETHER) {
                    interfaces.push_back(tmp->ifa_name);
                }
            }
#endif
        }
        tmp = tmp->ifa_next;
    }
    if (interfaces.empty()) {
        throw RuntimeException("No valid ETH interfaces found");
    } else {
        // std::cout << "Found ETH interfaces: ";
        // for (auto &s : interfaces) {
        //     std::cout << s << " ";
        // }
        // std::cout << std::endl;
    }
    freeifaddrs(addrs);
    return interfaces;
}

class L2File : public TextFile {
 public:
    L2File(std::string interface, mac_t dst_mac, L2MessageType cmd_type, L2MessageType cmd_status_type, L2MessageType status_type) : 
        cmd_type_(cmd_type), cmd_status_type_(cmd_status_type), status_type_(status_type),
        dst_mac_(dst_mac), interface_(interface) {
        

        // lock file to prevent multiple instances at the same time
        lock_file_ = "/tmp/obot." + interface_ + "-" + mac2str(dst_mac) + ".lock";
        fd_lock_ = ::open(lock_file_.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd_lock_ < 0) {
            throw RuntimeErrnoException("Error opening lock file " + lock_file_);
        }
        if (int retval = ::fchmod(fd_lock_, 0666); retval < 0) {
            if (errno != EPERM) {
                throw RuntimeErrnoException("Error setting permissions on lock file " + lock_file_);
            } // else ignore no permissions
        }
        if (int retval = ::lseek(fd_lock_, 0, SEEK_SET); retval < 0) {
            throw RuntimeErrnoException("Error lseek lock file " + lock_file_);
        }
        open();
    }

    void open() {
#ifdef __linux__
        fd_ = ::socket(AF_PACKET, SOCK_RAW, htons(0x88b5));
        if (fd_ < 0) {
            throw RuntimeErrnoException("socket failed for " + interface_);
        }

        if (fcntl(fd_, F_SETFL, O_NONBLOCK) < 0) {
            throw RuntimeErrnoException("socket set non-block failed for " + interface_);
        }


        // if (interface_ != "any") {
        //     if (setsockopt(fd_, SOL_SOCKET, SO_BINDTODEVICE, interface_.c_str(), interface_.size()) < 0) {
        //         throw RuntimeErrnoException("setsockopt SO_BINDTODEVICE error");
        //     }
        // }
#elif defined(__APPLE__)
        fd_ = bpf_open_interface(interface_);
#endif

        node_id_ = dst_mac_[5];
        TopicId topic_id {
            .node_id = node_id_ ,
            .bus_id = 0,
            .type = static_cast<uint16_t>(status_type_)
        };
        uint16_t topic_id_uint;
        std::memcpy(&topic_id_uint, &topic_id, sizeof(topic_id_uint));

        set_eth_packet_filter(fd_, dst_mac_, htons(topic_id_uint));
        flush();
#ifdef __linux__
        // still need to bind in order to send, I guess
        sockaddr_ll server_addr = {};
        server_addr.sll_family = AF_PACKET;
        server_addr.sll_protocol = htons(0x88B5);
        server_addr.sll_ifindex = if_nametoindex(interface_.c_str());

        int retval = bind(fd_, (struct sockaddr *)&server_addr, sizeof(server_addr));
        if (retval < 0) {
            throw RuntimeErrnoException("bind failed for " + interface_);
        }
#endif

        std::memcpy(&l2_frame_out_.dst_mac, &dst_mac_, sizeof(dst_mac_));
        mac_t src_mac = get_interface_mac_address(fd_, interface_);
        std::memcpy(&l2_frame_out_.src_mac, &src_mac, sizeof(src_mac));
    }

    void close() {
        ::close(fd_);
    }

    void flush() {
        // flush frames received before filter
        while (true) {
#ifdef __linux__
            char buf[MAX_ETH_L2_PAYLOAD_SIZE];
            int retval = ::read(fd_, buf, MAX_ETH_L2_PAYLOAD_SIZE);
#elif defined(__APPLE__)
            L2Frame tmp_frame;
            ssize_t retval = bpf_read_frame(&tmp_frame);
#endif
            if (retval < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                } else {
                    throw RuntimeErrnoException("read error during flush");
                }
            }
        }
    }

    void set_eth_packet_filter(int fd, mac_t mac, uint16_t topic_id) {
        uint32_t word1;
        std::memcpy(&word1, mac.data(), 4);
        word1 = htonl(word1);
        uint16_t word2;
        std::memcpy(&word2, mac.data()+4, 2);
        word2 = htons(word2);
        // Set Berkeley Packet Filter to only receive packets with mac matching.
        // The classic-BPF opcodes below are identical on Linux and BSD/macOS;
        // only the container struct and the attach mechanism differ.
#ifdef __linux__
        using insn_t = struct sock_filter;
#elif defined(__APPLE__)
        using insn_t = struct bpf_insn;
#endif
        insn_t bpf_code[] = {
            // Load first 4 bytes of Ethernet MAC
            { BPF_LD+BPF_W+BPF_ABS, 0, 0, 6 }, // BPF_LD+BPF_W+BPF_ABS = 0x20, offset 6
            // Compare with dst_mac_[0..3]
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 7, word1}, // BPF_JMP+BPF_JEQ+BPF_K = 0x15
            // Load next 2 bytes of Ethernet MAC
            { BPF_LD+BPF_H+BPF_ABS, 0, 0, 10 }, // BPF_LD+BPF_H+BPF_ABS = 0x28, offset 10
            // Compare with dst_mac_[4..5]
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 5, word2 }, // BPF_JMP+BPF_JEQ+BPF_K = 0x15
            // Check first 4 bytes of payload accept zero
            { BPF_LD+BPF_W+BPF_ABS, 0, 0, 14 },
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 3, 0 },
            // Check packet type field
            { BPF_LD+BPF_H+BPF_ABS, 0, 0, L2_HEADER_SIZE },
            { BPF_JMP+BPF_JEQ+BPF_K, 0, 1, htons(topic_id) },

            // Accept packet
            { BPF_RET+BPF_K, 0, 0, 0xFFFFFFFF }, // BPF_RET+BPF_K = 0x06, accept
            // Reject packet
            { BPF_RET+BPF_K, 0, 0, 0 }, // BPF_RET+BPF_K = 0x06, drop
        };

#ifdef __linux__
        struct sock_fprog bpf_prog = {
            .len = sizeof(bpf_code)/sizeof(bpf_code[0]),
            .filter = bpf_code,
        };
        if (int result = setsockopt(fd, SOL_SOCKET, SO_ATTACH_FILTER, &bpf_prog, sizeof(bpf_prog)); result < 0) {
            throw RuntimeErrnoException("Failed to set BPF filter");
        }
#elif defined(__APPLE__)
        struct bpf_program bpf_prog;
        bpf_prog.bf_len = sizeof(bpf_code)/sizeof(bpf_code[0]);
        bpf_prog.bf_insns = bpf_code;
        if (ioctl(fd, BIOCSETF, &bpf_prog) < 0) {
            throw RuntimeErrnoException("Failed to set BPF filter (BIOCSETF)");
        }
#endif
    }

    // use a lock file to provide exclusive access to the device during a 
    // write followed by read interface to the text api
    int lock() {
        int err = lockf(fd_lock_, F_LOCK, 1);
        if (err) {
            std::cerr << "error locking " + lock_file_;
            pid_t pid;
            int err2 = get_lock_pid(fd_lock_, &pid);
            if (err2 == 0) {
                std::cerr << ", already locked by process: " << pid;
            }
            std::cerr << std::endl;
        }
        // remove all extra packets from other processes
        flush();
        return err;
    }

    int unlock() {
        // close();
        int err = lockf(fd_lock_, F_ULOCK, 0);
        if (err) {
            std::cerr << "error unlocking " + lock_file_;
            pid_t pid;
            int err2 = get_lock_pid(fd_lock_, &pid);
            if (err2 == 0) {
                std::cerr << ", locked by process: " << pid;
            }
            std::cerr << std::endl;
        }
        return err;
    }

    ssize_t _read(char * data, unsigned int length, bool request = false, bool flush = true) {
        if (request) {
            lock();
            TopicId topic_id {
                .node_id = node_id_ ,
                .bus_id = 0,
                .type = static_cast<uint16_t>(status_type_)
            };
            uint16_t topic_id_uint;
            std::memcpy(&topic_id_uint, &topic_id, sizeof(topic_id));
            Payload payload {
                .topic_id = htons(topic_id_uint),
            };
            std::memset(&l2_frame_out_.payload, 0, 64-L2_HEADER_SIZE);
            std::memcpy(l2_frame_out_.payload, &payload, PAYLOAD_HEADER_SIZE);
            int length_out = 64;
            int result = raw_send(&l2_frame_out_, length_out);
            if (result < 0) {
                std::cout << RuntimeHexDumpException::hex_dump((uint8_t *) &l2_frame_out_, length_out) << std::endl;
                throw RuntimeErrnoException("eth write error");
            }
        }
        pollfd poll_fd {
            .fd = fd_,
            .events = POLLIN
        };

        int nbytes = -1;
        L2Frame frame {};
        // flush but save read frame
        while (flush) {
            L2Frame tmp_frame;
            int tmp_nbytes = raw_recv(&tmp_frame);
            if (tmp_nbytes < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                } else {
                    throw RuntimeErrnoException("read error during flush " + name());
                }
            } else {
                frame = tmp_frame;
                nbytes = tmp_nbytes;
            }
        }

        // if nothing from the no timeout flush/read, then do a timeout read
        if (nbytes < 0) {
            if (int poll_result = poll(&poll_fd, 1, timeout_ms_); poll_result < 0) {
                throw RuntimeErrnoException("poll error in read " + name());
            } else if (poll_result == 0) {
                throw RuntimeException("poll timeout in read " + name());
            }

            nbytes = raw_recv(&frame);
        }

        if (request) {
            unlock();
        }
        if (nbytes <= 0) {
            throw RuntimeErrnoException("eth read error " + name());
        }
    
        Payload payload {};
        std::memcpy(&payload, frame.payload, PAYLOAD_HEADER_SIZE);
        payload.topic_id = ntohs(payload.topic_id);
        payload.length = ntohs(payload.length);
        if (payload.length > MAX_PAYLOAD_LENGTH || payload.length > nbytes - L2_HEADER_SIZE - PAYLOAD_HEADER_SIZE) {
            throw RuntimeHexDumpException("payload length error: " + std::to_string(payload.length), reinterpret_cast<uint8_t*>(&frame), nbytes);
        }
        length = std::min(static_cast<unsigned int>(payload.length), length);
        std::memcpy(data, &frame.payload[PAYLOAD_HEADER_SIZE], length);
        return length;
    }

    virtual ssize_t read(char * data, unsigned int length) {
        ssize_t retval = _read(data, length);

        if (retval >= sizeof(APIControlPacket) && data[0] == 0) {
            // a control packet
            APIControlPacket * packet = reinterpret_cast<APIControlPacket *>(data);
            if (packet->type == TIMEOUT_REQUEST) {
                // timeout request
                if (retval == sizeof(APIControlPacket)) {
                    // timeout request
                    // retriggers the read with the new timeout
                    uint32_t old_timeout_ms = timeout_ms_;
                    timeout_ms_ += packet->timeout_request.timeout_us/1000;
                    ssize_t retval = read(data, length);
                    timeout_ms_ = old_timeout_ms;
                    return retval;
                }
            } else if (packet->type == LONG_PACKET) {
                // long packet
                uint16_t total_length = packet->long_packet.total_length;
                uint16_t packet_number = packet->long_packet.packet_number;
                const uint8_t header_size = sizeof(APIControlPacket);
                uint16_t total_count_received = retval - header_size;
                if (total_length > length) {
                    // too long
                    return -EINVAL;
                }
                std::memmove(data, data + header_size, total_count_received);
                while (total_length > total_count_received) {
                    // assemble multiple packets
                    char * data_ptr = data + total_count_received;
                    retval = _read(data_ptr, length, false, false);
                    if (retval < 0) {
                        return retval;
                    }
                    APIControlPacket * packet = reinterpret_cast<APIControlPacket *>(data_ptr);
                    if (packet->type != LONG_PACKET) {
                        std::cerr << "Error: expected long packet, got " << packet->type << std::endl;
                        return -EINVAL;
                    }
                    if (packet->long_packet.packet_number != ++packet_number) {
                        std::cerr << "Error: expected packet number " << packet_number << ", got " << packet->long_packet.packet_number << std::endl;
                        return -EINVAL;
                    }
                    //std::cout << packet->long_packet.packet_number << retval << std::endl;
                    total_count_received += retval - header_size;
                    std::memmove(data_ptr, data_ptr + header_size, retval - header_size);
                }
                if (total_count_received < total_length) {
                    std::cerr << "Error: expected " << total_length << " bytes, got " << total_count_received << std::endl;
                    return -EINVAL;
                }
                retval = total_count_received;
            }
        } else if (retval > 0) {
            // a string
            retval = strnlen(data, length);
        }
        unlock();
        return retval;
    }


    virtual ssize_t write(const char * data, unsigned int length, bool writeread = false) {
        lock();
        L2MessageType cmd_type = writeread ? cmd_status_type_ : cmd_type_;
        TopicId topic_id {
            .node_id = node_id_ ,
            .bus_id = 0,
            .type = static_cast<uint16_t>(cmd_type)
        };
        uint16_t topic_id_uint;
        std::memcpy(&topic_id_uint, &topic_id, sizeof(topic_id));
        Payload payload {
            .topic_id = htons(topic_id_uint),
            .length = htons(length),
        };
        std::memset(&l2_frame_out_.payload, 0, 64-L2_HEADER_SIZE);
        std::memcpy(l2_frame_out_.payload, &payload, PAYLOAD_HEADER_SIZE);
        length = std::min(length, static_cast<unsigned int>(MAX_ETH_L2_PAYLOAD_SIZE));
        std::memcpy(l2_frame_out_.payload+PAYLOAD_HEADER_SIZE, data, length);
        int length_out = std::max(length+PAYLOAD_HEADER_SIZE+L2_HEADER_SIZE, 64u);
        int result = raw_send(&l2_frame_out_, length_out);
        if (result < 0) {
            std::cout << RuntimeHexDumpException::hex_dump((uint8_t *) &l2_frame_out_, length_out) << std::endl;
            throw RuntimeErrnoException("eth write error " + name());
        }
        return length_out;
    }

    virtual ssize_t writeread(const char * data_out, unsigned int length_out, char * data_in, unsigned int length_in) {
        //int err = lock();
        // if (err) {
        //     return err;
        // }
        ssize_t nbytes = write(data_out, length_out, true);
        if (nbytes < 0) {
            return nbytes;
        }
        int retval = read(data_in, length_in);
        //err = unlock();
        // if (err) {
        //     return err;
        // }
        return retval;
    }

    std::string name() const {
        return interface_ + "-" + mac2str(dst_mac_);
    }

    // --- platform raw-frame backend ----------------------------------------
#ifdef __APPLE__
    // macOS BPF backend: open /dev/bpfN bound to the interface in immediate
    // (per-packet), non-blocking mode, supplying the full link header on TX.
    int bpf_open_interface(const std::string& ifname) {
        int fd = -1;
        for (int i = 0; i < 256; i++) {
            std::string path = "/dev/bpf" + std::to_string(i);
            fd = ::open(path.c_str(), O_RDWR);
            if (fd >= 0) {
                break;
            }
            if (errno == ENOENT) {
                break;  // no more bpf devices to try
            }
            // EBUSY: device in use, try the next one
        }
        if (fd < 0) {
            throw RuntimeErrnoException("could not open a /dev/bpf* device for " + ifname);
        }
        struct ifreq ifr = {};
        std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
        if (ioctl(fd, BIOCSETIF, &ifr) < 0) {
            ::close(fd);
            throw RuntimeErrnoException("BIOCSETIF failed for " + ifname);
        }
        unsigned int blen = 0;
        if (ioctl(fd, BIOCGBLEN, &blen) < 0) {
            ::close(fd);
            throw RuntimeErrnoException("BIOCGBLEN failed for " + ifname);
        }
        bpf_buf_.resize(blen);
        int enable = 1;
        int disable = 0;
        ioctl(fd, BIOCIMMEDIATE, &enable);   // deliver packets immediately
        ioctl(fd, BIOCSHDRCMPLT, &enable);   // we provide the source MAC ourselves
        ioctl(fd, BIOCSSEESENT, &disable);   // don't echo back our own transmits
        if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0) {
            ::close(fd);
            throw RuntimeErrnoException("bpf set non-block failed for " + ifname);
        }
        return fd;
    }

    // De-frame one Ethernet frame out of the BPF read buffer. A single read()
    // may return several packets, so leftover bytes are parsed on later calls.
    // Returns the frame length (>0), or -1 with errno=EAGAIN when nothing is
    // buffered (mirrors Linux ::read on a non-blocking AF_PACKET socket).
    ssize_t bpf_read_frame(L2Frame* out) {
        if (bpf_offset_ >= static_cast<size_t>(bpf_valid_)) {
            ssize_t n = ::read(fd_, bpf_buf_.data(), bpf_buf_.size());
            if (n <= 0) {
                if (n == 0) {
                    errno = EAGAIN;
                }
                return -1;
            }
            bpf_valid_ = n;
            bpf_offset_ = 0;
        }
        struct bpf_hdr* bh = reinterpret_cast<struct bpf_hdr*>(bpf_buf_.data() + bpf_offset_);
        uint8_t* pkt = bpf_buf_.data() + bpf_offset_ + bh->bh_hdrlen;
        size_t copy_len = std::min(static_cast<size_t>(bh->bh_caplen), sizeof(L2Frame));
        std::memset(out, 0, sizeof(L2Frame));
        std::memcpy(out, pkt, copy_len);
        bpf_offset_ += BPF_WORDALIGN(bh->bh_hdrlen + bh->bh_caplen);
        return static_cast<ssize_t>(copy_len);
    }

    ssize_t raw_send(const void* buf, size_t len) { return ::write(fd_, buf, len); }
    ssize_t raw_recv(L2Frame* frame) { return bpf_read_frame(frame); }

    std::vector<uint8_t> bpf_buf_;
    size_t bpf_offset_ = 0;
    ssize_t bpf_valid_ = 0;
#else
    ssize_t raw_send(const void* buf, size_t len) { return ::send(fd_, buf, len, 0); }
    ssize_t raw_recv(L2Frame* frame) { return ::read(fd_, frame, sizeof(*frame)); }
#endif

    int fd_;
    L2MessageType cmd_type_, cmd_status_type_, status_type_;
    std::string interface_;
    mac_t dst_mac_;
    int timeout_ms_ = 10;
    int fd_lock_;
    L2Frame l2_frame_out_;
    uint8_t node_id_;
    
    std::string lock_file_;
};

MotorEthL2::MotorEthL2(std::string address, std::string alias) {
    std::string interface;
    std::string mac;
    if (int n = address.find("-"); n == std::string::npos) {
        interface = "any";
        mac = address;
    } else {
        interface = address.substr(0, n);
        mac = address.substr(n+1,-1);
    }
    mac_t dst_mac = str2mac(mac);
    devnum_ = dst_mac[5];
    dev_path_ = mac;

    if (interface == "any") {
        auto interfaces = get_eth_interfaces();
        for (auto & tmp_interface : interfaces) {
            auto motor_txt = L2File(tmp_interface, dst_mac, L2MessageType::OBOT_ASCII_CMD, L2MessageType::OBOT_ASCII_CMD, L2MessageType::OBOT_ASCII_RESPONSE);
            try {
                std::string str_out = "messages_version";
                char str_in[64];
                motor_txt.writeread(str_out.c_str(), str_out.length(), str_in, 64);
                interface = tmp_interface;
                break;
            } catch (RuntimeException &e) {}
        }
        if (interface == "any") {
            throw RuntimeException(mac + " not found");
        }
    }

    base_path_ = interface;

    realtime_file_ = std::unique_ptr<L2File>(new L2File(interface, dst_mac, L2MessageType::OBOT_CMD, L2MessageType::OBOT_CMD_STATUS, L2MessageType::OBOT_STATUS));
    open();

    motor_txt_ = std::move(std::unique_ptr<L2File>(new L2File(interface, dst_mac, L2MessageType::OBOT_ASCII_CMD, L2MessageType::OBOT_ASCII_CMD, L2MessageType::OBOT_ASCII_RESPONSE)));
	
    messages_version_ = operator[]("messages_version").get();
    name_ = operator[]("name").get();
    version_ = operator[]("version").get();
    board_name_ = operator[]("board_name").get();
    board_rev_ = operator[]("board_rev").get();
    board_num_ = operator[]("board_num").get();
    config_ = operator[]("config").get();
    serial_number_ = operator[]("serial").get();
    connected_ = true;
}

MotorEthL2::~MotorEthL2() {}

uint32_t MotorEthL2::timeout_ms_ = 10;
void MotorEthL2::set_timeout_ms(int timeout_ms) {
    timeout_ms_ = timeout_ms;
    static_cast<L2File*>(motor_txt_.get())->timeout_ms_ = timeout_ms;
}

void MotorEthL2::open() {
    
}


ssize_t MotorEthL2::read() {
    L2Frame frame_in;
    int nbytes = 0;
    if (int retval = realtime_file_->_read(reinterpret_cast<char *>(&frame_in), sizeof(frame_in), send_read_request_);
        retval < 0) {
        throw RuntimeErrnoException("Error on EthL2 read");
    } else {
        nbytes = std::min(retval, static_cast<int>(sizeof(status_)));
        std::memcpy(&status_, &frame_in, nbytes);
    }
    return nbytes;
}

ssize_t MotorEthL2::write() {
    int nbytes = 0;
    if (int retval = realtime_file_->write(reinterpret_cast<const char*>(&command_), sizeof(command_), cmd_status_req_);
        retval < 0) {
        throw RuntimeErrnoException("Error on EthL2 write");
    } else {
        nbytes = retval;
    }
    return nbytes;
}

}; // namespace obot

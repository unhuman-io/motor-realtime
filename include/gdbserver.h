#include <functional>
#include <atomic>

namespace obot {

class GDBServer {
    public:
        GDBServer(std::function<std::string(std::string_view)> writeread, std::atomic<bool> &signal_exit);
        ~GDBServer();
        void start();
        void send_ack() {send_response("+");}
        void send_nack() {send_response("-");}
        void send_response(std::string response);
        void send_gdb_packet(std::string response);
        void periodically_check_status();
    private:
        std::function<std::string(std::string_view)> writeread_;
        std::atomic<bool> &signal_exit_;
        int connfd_ = 0;
        int sockfd = 0;
};

}

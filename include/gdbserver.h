#include <functional>

namespace obot {

class GDBServer {
    public:
        GDBServer(std::function<std::string(std::string_view)> writeread);
        ~GDBServer();
        void start();
        void send_ack() {send_response("+");}
        void send_nack() {send_response("-");}
        void send_response(std::string response);
    private:
        std::function<std::string(std::string_view)> writeread_;
        int connfd_;
        int sockfd;
};

}

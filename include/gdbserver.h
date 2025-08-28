
namespace obot {

class GDBServer {
    public:
        GDBServer() {}
        void start();
        void send_ack() {send_response("+");}
        void send_nack() {send_response("-");}
        void send_response(std::string response);
    private:
        int connfd_;
};

}

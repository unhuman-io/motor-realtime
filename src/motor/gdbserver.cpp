
#include <netdb.h> 
#include <netinet/in.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#include <sys/types.h> 
#include <unistd.h>
#include <iostream>
#include <string_view>
#include <thread>
#include <chrono>

// in gdb
// set debug remote 1
// target remote :3293

const int PORT = 3293;

#include "gdbserver.h"

namespace obot {
void GDBServer::start() {
    int connfd, len; 
    struct sockaddr_in cli; 
    
    int sockfd = socket(AF_INET, SOCK_STREAM, 0); 
    if (sockfd == -1) { 
        throw std::runtime_error("socket creation failed");
    }

    std::cout << "socket created successfully" << std::endl;
    struct sockaddr_in servaddr{};
    
    servaddr.sin_family = AF_INET; 
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY); 
    servaddr.sin_port = htons(PORT); 
    
    if ((bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr))) != 0) { 
        throw std::runtime_error("socket bind failed"); 
    } 
    std::cout << "socket successfully bound" << std::endl; 
    
    if ((listen(sockfd, 0)) != 0) { 
        throw std::runtime_error("Listen failed"); 
    } 
    std::cout << "server listening" << std::endl;; 
    len = sizeof(cli); 
    
    connfd = accept(sockfd, (struct sockaddr*)&cli, (socklen_t*) &len); 
    if (connfd < 0) { 
        throw std::runtime_error("server accept failed..."); 
    }
    std::cout << "server accepted the client" << std::endl;

    const int MAX=1000;
    char buf[MAX]; 
    for (;;) {
        bzero(buf, MAX); 
    
        int reval = read(connfd, buf, sizeof(buf)); 
        std::cout << "read result " << reval << std::endl;
        if (reval < 0) {
            throw std::runtime_error("read error");
        }
        if (reval == 0) {
            throw std::runtime_error("read closed pipe");
        }


        std::cout << "From gdb: " <<  buf << std::endl;
        std::string response;
        std::string_view str(buf);
        bool ack;
        if (str.rfind("$", 0) == 0) {
            ack = true;
            response = "+";
        } else if (str.rfind("+", 0) == 0) {
            //ignore
            ack = false;
            response = "";
        } else {
            //else "-"
            ack = false;
            response = "-";
        }
        if (response.size() > 0) {
            std::cout << "sending gdb response: " << response << std::endl;
            int n = write(connfd, response.c_str(), response.size());
            if (n < 0) {
                throw std::runtime_error("write error");
            }
        }
        if (!ack) {
            continue;
        }
        if (str.rfind("$qSupported", 0) == 0) {
            std::cout << "gdb command: " << str.substr(1) << std::endl;
            response = "read+;write+;";
        } else if (str.rfind("$g", 0) == 0) {
            std::cout << "gdb command: " << str.substr(1) << std::endl;
            response = "00";
        } else {
            response = "";
        }

        uint8_t checksum = 0;
        for (size_t i = 0; i < response.size(); i++) {
            checksum += response[i];
        }
        char checksum_str[3];
        snprintf(checksum_str, sizeof(checksum_str), "%02x", checksum);
        std::string gdb_response = "$" + response + "#" + checksum_str;
        std::cout << "gdb response: " << gdb_response << std::endl;
        int n = write(connfd, gdb_response.c_str(), gdb_response.size());
        if (n < 0) {
            throw std::runtime_error("write error");
        }
        std::cout << "write result " << n << std::endl;
        if (n == 0) {
            std::cout << "write closed pipe" << std::endl;
            break;
        }
        //std::this_thread::sleep_for(std::chrono::milliseconds(5000));

         
    } 
    close(sockfd);
}
} // namespace obot

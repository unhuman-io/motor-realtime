
#include <netdb.h> 
#include <netinet/in.h> 
#include <stdlib.h> 
#include <string.h> 
#include <string>
#include <sys/socket.h> 
#include <sys/types.h> 
#include <unistd.h>
#include <iostream>
#include <string_view>
#include <thread>
#include <chrono>
#include "exception.h"

// in gdb
// set debug remote 1
// target remote :3293

const int PORT = 3293;

#include "gdbserver.h"

namespace obot {

GDBServer::GDBServer(std::function<std::string(std::string_view)> writeread) : writeread_(writeread) {
    int len; 
    struct sockaddr_in cli; 
    
    sockfd = socket(AF_INET, SOCK_STREAM, 0); 
    if (sockfd == -1) { 
        throw RuntimeException("socket creation failed");
    }
    int option = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

    std::cout << "socket created successfully" << std::endl;
    struct sockaddr_in servaddr{};
    
    servaddr.sin_family = AF_INET; 
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY); 
    servaddr.sin_port = htons(PORT); 
    
    if ((bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr))) != 0) { 
        throw RuntimeException("socket bind failed: " + std::string(strerror(errno))); 
    } 
    std::cout << "socket successfully bound" << std::endl; 
    
    if ((listen(sockfd, 0)) != 0) { 
        throw RuntimeException("Listen failed"); 
    } 
    std::cout << "server listening" << std::endl;

    len = sizeof(cli);
        connfd_ = accept(sockfd, (struct sockaddr*)&cli, (socklen_t*) &len); 
    if (connfd_ < 0) { 
        throw RuntimeException("server accept failed..."); 
    }
    std::cout << "server accepted the client" << std::endl;

}

GDBServer::~GDBServer() {
    std::cout << "closing gdb server sockets" << std::endl;
    close(connfd_);
    close(sockfd);
}

void GDBServer::send_response(std::string response) {
    std::cout << "sending response: " << response << std::endl;
    int n = write(connfd_, response.c_str(), response.size());
    if (n < 0) {
        throw RuntimeException("write error");
    }
}

void GDBServer::start() {
    const int MAX=1000;
    char buf[MAX]; 
    for (;;) {
        bzero(buf, MAX); 
    
        int reval = read(connfd_, buf, sizeof(buf)); 
        std::cout << "read result " << reval << std::endl;
        if (reval < 0) {
            throw RuntimeException("read error");
        }
        if (reval == 0) {
            throw RuntimeException("read closed pipe");
        }


        std::cout << "From gdb: " <<  buf << std::endl;
        std::string response;
        std::string_view str(buf);
        bool ack = false;
        if (str.rfind("$", 0) == 0) {
            send_ack();
            ack = true;
        } else if (str.rfind("+", 0) == 0) {
            //ignore
            str.remove_prefix(1);
            if (str.rfind("$", 0) == 0) {
                send_ack();
                ack = true;
            }
        } else if (str.rfind("\x003", 0) == 0) {
            //interrupt
            send_ack();
            ack = true;
        } else {
            //else "-"
            send_nack();
        }
        if (!ack) {
            continue;
        }

        if (str.rfind("$qSupported", 0) == 0) {
            std::cout << "gdb command: " << str.substr(1) << std::endl;
            response = "read+;write+";//;QNonStop+";
        } else if (str.rfind("$g", 0) == 0) {
            std::cout << "gdb command: " << str.substr(1) << std::endl;
            auto item = writeread_("$g");
            response = item;
            //response = std::string(17*4*2, '0');
            //response = "000000004aff7f40000000000000000000000000000000000000000000000000000000000000000044f10b000000000000000000a0fd7f400000000038ab000000000001";
        } else if (str.rfind("$?", 0) == 0) {
            std::cout << "gdb command: " << str.substr(1) << std::endl;
            response = "S05"; // "T05thread:pbdeab.bdeab;"
        } else if (str.rfind("$Hc-1", 0) == 0) {
            std::cout << "gdb command: " << str.substr(1) << std::endl;
            response = "OK";
        } else if (str.rfind("$qAttached", 0) == 0) {
            std::cout << "gdb command: " << str.substr(1) << std::endl;
            response = "1";
        } else if (str.rfind("$qOffsets", 0) == 0) {
            std::cout << "gdb command: " << str.substr(1) << std::endl;
            response = "Text=00000000;Data=00000000;Bss=00000000";
        } else if (str.rfind("$mbf284", 0) == 0) {
            std::cout << "gdb command: " << str.substr(1) << std::endl;
            response = "12345678";
        } else if (str.rfind("$m", 0) == 0) {
            std::string_view memory_read = str.substr(0, str.size() - 3);
            std::cout << "gdb command: " << memory_read << std::endl;
            response = writeread_(memory_read);
        } else if (str.rfind("$M", 0) == 0) {
            std::string_view memory_write = str.substr(0, str.size() - 3);
            std::cout << "gdb command: " << memory_write << std::endl;
            response = writeread_(memory_write);
        } else if (str.rfind("$P", 0) == 0) {
            std::string_view reg_write = str.substr(0, str.size() - 3);
            std::cout << "gdb reg write command: " << reg_write << std::endl;
            response = writeread_(reg_write);
        } else if (str.rfind("$QNonStop", 0) == 0) {
            std::cout << "gdb non stop command: " << str.substr(0) << std::endl;
            response = "OK";
        } else if (str.rfind("$c", 0) == 0) {
            std::cout << "gdb c command: " << str.substr(0) << std::endl;
            auto item = writeread_("$c");
            std::cout << "item response: " << item << std::endl;
            // no response
            continue;
        } else if (str.rfind("\x003", 0) == 0) {
            std::cout << "gdb interrupt command" << std::endl;
            std::string_view interrupt = "$b";
            auto item = writeread_(interrupt);
            std::cout << "item response: " << item << std::endl;
            response = "S05";
        } else if (str.rfind("$s", 0) == 0) {
            std::cout << "gdb si command" << std::endl;
            auto item = writeread_("$s");
            std::cout << "item response: " << item << std::endl;
            response = "S05";
        } else if (str.rfind("$p", 0) == 0) {
            std::cout << "gdb p command" << str.substr(1) << std::endl;
            response = writeread_(str.substr(0, str.size() - 3));
        } else if (str.rfind("$Z", 0) == 0) {
            std::cout << "gdb breakpoint command" << std::endl;
            auto item = writeread_(str.substr(0, str.size() - 3));
            std::cout << "item response: " << item << std::endl;
            response = "OK";
        } else if (str.rfind("$z", 0) == 0) {
            std::cout << "gdb remove breakpoint command" << std::endl;
            auto item = writeread_(str.substr(0, str.size() - 3));
            std::cout << "item response: " << item << std::endl;
            response = "OK";
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
        int n = write(connfd_, gdb_response.c_str(), gdb_response.size());
        if (n < 0) {
            throw RuntimeException("write error");
        }
        std::cout << "write result " << n << std::endl;
        if (n == 0) {
            std::cout << "write closed pipe" << std::endl;
            break;
        }
        //std::this_thread::sleep_for(std::chrono::milliseconds(5000));

         
    } 

}
} // namespace obot

#include "rt_version.h"
#include "CLI11.hpp"
#include "motor_manager.h"
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include "motor_server.grpc.pb.h"

class MotorServerImpl final : public MotorServer::Service {
 public:
    grpc::Status GetMotorInfo(grpc::ServerContext* context, const MotorInfoRequest* request, MotorInfoResponse* reply) override {
        for (auto m : m_.get_connected_motors()) {
            auto motor_info = reply->add_motor_info();
            motor_info->set_name(m->name());
            motor_info->set_devpath(m->dev_path());
            motor_info->set_path(m->base_path());
            motor_info->set_serial_number(m->serial_number());
            motor_info->set_version(m->version());
        }
        return grpc::Status::OK;
    }
 private:
    MotorManager m_;
};

int main(int argc, char** argv) {
    CLI::App app{"gRPC network interface to motors"};
    CLI11_PARSE(app, argc, argv);

    std::string server_address("0.0.0.0:50051");
    MotorServerImpl service;

    grpc::EnableDefaultHealthCheckService(true);
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    grpc::ServerBuilder builder;
    // Listen on the given address without any authentication mechanism.
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Register "service" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *synchronous* service.
    builder.RegisterService(&service);
    // Finally assemble the server.
    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    std::cout << "Server listening on " << server_address << std::endl;

    // Wait for the server to shutdown. Note that some other thread must be
    // responsible for shutting down the server for this call to ever return.
    server->Wait();

    return 0;
}
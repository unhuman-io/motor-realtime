#!/usr/bin/env python

from __future__ import print_function
import logging

import grpc

import motor_server_pb2
import motor_server_pb2_grpc

# note:
# python -m grpc_tools.protoc -I../protos --python_out=. --grpc_python_out=. ../protos/motor_server.proto



def run():
    # NOTE(gRPC Python Team): .close() is possible on a channel and should be
    # used in circumstances in which the with statement does not fit the needs
    # of the code.
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = motor_server_pb2_grpc.MotorServerStub(channel)
        response = stub.GetMotorInfo(motor_server_pb2.MotorInfoRequest(name='you'))
        print(str(len(response.motor_info)) + " motors connected")
        for info in response.motor_info:
            print(" ".join([info.name, info.devpath, info.path, info.serial_number, info.version]))


if __name__ == '__main__':
    logging.basicConfig()
    run()

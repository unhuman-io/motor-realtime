#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "motor_publisher.h"
#include "motor_subscriber.h"

namespace py = pybind11;
using namespace obot;

PYBIND11_MODULE(motor_pub_sub, m)
{
    m.doc() = "Motor publisher and subscriber example";

    struct Data {
        double pos_x;
        double pos_y;
        double pos_z;
    };
    struct Command{
        double force_x;
        double force_y;
        double force_z;
    };

    py::class_<Data>(m, "Data")
        .def(py::init())
        .def_readwrite("pos_x", &Data::pos_x)
        .def_readwrite("pos_y", &Data::pos_y)
        .def_readwrite("pos_z", &Data::pos_z);

    py::class_<Command>(m, "Command")
        .def(py::init())
        .def_readwrite("force_x", &Command::force_x)
        .def_readwrite("force_y", &Command::force_y)
        .def_readwrite("force_z", &Command::force_z);

    py::class_<MotorPublisher<Command>>(m, "MotorPublisher")
        .def(py::init<std::string>())
        .def("publish", &MotorPublisher<Command>::publish);

    py::class_<MotorSubscriber<Data>>(m, "MotorSubscriber")
        .def(py::init<std::string>())
        .def("read", &MotorSubscriber<Data>::read);
}

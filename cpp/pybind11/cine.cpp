#include <pybind11/pybind11.h>

#include "CineReader.h"

namespace py = pybind11;

PYBIND11_MODULE(cine, m) {
    m.doc() = "Phantom Vision *.cine video file interface";

    py::class_<upsp::CineReader>(m, "CineReader")
        .def(py::init<std::string>());
}

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "utils/pspRT.h"

namespace py = pybind11;

PYBIND11_MODULE(raycast, m)
{
    m.doc() = "Ray-tracing capabilities using a Bounding Volume Hierarchy (BVH)";

    m.def(
        "CreateBVH",
        &rt::CreateBVH,
        "Create a bounding volume hierarchy");

    py::class_<rt::Ray>(m, "Ray")
        .def(py::init([](const float x0, const float y0, const float z0,
                         const float xr, const float yr, const float zr) {
            Imath::V3f o(x0, y0, z0);
            Imath::V3f d(xr, yr, zr);
            return std::make_unique<rt::Ray>(o, d);
        }));

    py::class_<rt::Hit>(m, "Hit")
        .def(py::init<>())
        .def_property_readonly("pos", [](const rt::Hit &hit) {
            std::vector<float> v = {hit.pos.x,
                                    hit.pos.y,
                                    hit.pos.z};
            return v;
        });

    py::class_<rt::BVH>(m, "BVH")
        .def(py::init<const std::vector<std::shared_ptr<rt::Primitive>> &, int>())
        .def("intersect", &rt::BVH::intersect);
}

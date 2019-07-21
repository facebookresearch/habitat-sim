#include <gl_tensor.h>
#include <gl_tensor_param.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

using namespace gltensor;

PYBIND11_MODULE(gl_tensor, m) {
  m.doc() = R"pbdoc(
        Graphic-Cuda PyTorch Tensor plugin
        -----------------------
        .. currentmodule:: gl_tensor
        .. autosummary::
           :toctree: _generate
    )pbdoc";

  py::class_<GLTensorParam, GLTensorParam::ptr>(m, "GLTensorParam")
      .def_readonly("device_id", &GLTensorParam::device_id_)
      .def_readonly("width", &GLTensorParam::width_)
      .def_readonly("height", &GLTensorParam::height_)
      .def_readonly("channels", &GLTensorParam::channels_);

  py::class_<GLTensor>(m, "GLTensor")
      .def("tensor", &GLTensor::Tensor)
      .def("release", &GLTensor::release);

  m.def("CudaTensor", &GLTensor::CreateCudaTensor,
        "A function which returns cuda tensor with direct memory mapping");

#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif
}

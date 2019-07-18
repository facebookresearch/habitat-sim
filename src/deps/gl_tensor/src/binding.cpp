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

  py::class_<GLTensorParam, GLTensorParam::ptr>(m, "GLTensorParam");

  py::class_<GLTensor>(m, "GLTensor")
      .def("Tensor", &GLTensor::Tensor)
      .def("Data", &GLTensor::Data,
           py::return_value_policy::reference_internal);
  m.def("CudaTensor", &GLTensor::CreateCudaTensor,
        "A function which returns cuda tensor with direct memory mapping");
  m.def(
      "Cpu2CudaTensor", &GLTensor::CreateCpu2CudaTensor,
      "A function which returns cuda tensor with gpu->cpu->gpu memory copying");
  m.def("CpuTensor", &GLTensor::CreateCpuTensor,
        "A function which returns cpu tensor");

#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif
}

#include <gl_tensor.h>

#include <cuda_gl_interop.h>
#include <cuda_runtime.h>
#include <helper_cuda.h>

#include <torch/extension.h>

namespace gltensor {

GLTensor::GLTensor(const GLTensorParam::ptr param, at::Tensor tensor)
    : image_(param->image_),
      width_(param->width_),
      height_(param->height_),
      channels_(param->channels_),
      data_ptr_(nullptr),
      tensor_(tensor) {
  assert(image_ != 0);
  assert(width_ > 0 && height_ > 0);
  size_ = channels_ * width_ * height_;

  switch (param->format_) {
    case GL_RGBA:
      type_ = at::ScalarType::Byte;
      break;
    case GL_R32F:
      type_ = at::ScalarType::Float;
      size_ *= sizeof(float);
      break;
    case GL_R32UI:
      type_ = at::ScalarType::Int;
      size_ *= 4;
      break;

    default:
      throw std::runtime_error("GLTensor unknown format");
  }
}

GLTensor::~GLTensor() {
  tensor_.reset();
}

at::Tensor GLTensor::Tensor() {
  Update();
  return tensor_;
}

class CudaTensor : public GLTensor {
 private:
  unsigned int target_;
  int device_id_;
  cudaGraphicsResource* cuda_graphics_resource_ = nullptr;

 public:
  CudaTensor(const GLTensorParam::ptr param, at::Tensor tensor)
      : GLTensor(param, tensor),
        target_{param->target_},
        device_id_{param->device_id_} {
    assert(param->device_id_ >= 0);
    assert(param->target_ == GL_RENDERBUFFER ||
           param->target_ == GL_TEXTURE_2D);

    checkCudaErrors(cudaSetDevice(param->device_id_));

    checkCudaErrors(
        cudaGraphicsGLRegisterImage(&cuda_graphics_resource_, image_, target_,
                                    cudaGraphicsRegisterFlagsReadOnly));
  }

  virtual void release() {
    if (cuda_graphics_resource_ != nullptr) {
      checkCudaErrors(cudaGraphicsUnregisterResource(cuda_graphics_resource_));
      cuda_graphics_resource_ = nullptr;
    }
  }

  virtual ~CudaTensor() { release(); }

 protected:
  void Update() {
    checkCudaErrors(cudaGraphicsMapResources(1, &cuda_graphics_resource_, 0));

    cudaArray* array = nullptr;
    checkCudaErrors(cudaGraphicsSubResourceGetMappedArray(
        &array, cuda_graphics_resource_, 0, 0));
    checkCudaErrors(cudaMemcpyFromArray(tensor_.data_ptr(), array, 0, 0, size_,
                                        cudaMemcpyDeviceToDevice));

    checkCudaErrors(cudaGraphicsUnmapResources(1, &cuda_graphics_resource_, 0));
  }
};

GLTensor* GLTensor::CreateCudaTensor(const GLTensorParam::ptr param, at::Tensor tensor) {
  return new CudaTensor(param, tensor);
}

}  // namespace gltensor

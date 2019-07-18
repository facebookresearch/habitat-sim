#include <gl_tensor.h>

#include <glad/glad.h>
#include <glad/glad_egl.h>

#include <cuda_gl_interop.h>
#include <cuda_runtime.h>
#include <helper_cuda.h>
#include <helper_cuda_gl.h>

#include <torch/torch.h>

namespace gltensor {

GLTensor::GLTensor(const GLTensorParam::ptr param)
    : image_(param->image_),
      width_(param->width_),
      height_(param->height_),
      data_ptr_(nullptr) {
  assert(image_ != 0);
  assert(width_ > 0 && height_ > 0);
  size_ = 4 * width_ * height_;
}

GLTensor::~GLTensor() { tensor_.reset(); }

at::Tensor GLTensor::Tensor() {
  Update();
  return tensor_;
}

class CudaTensor : public GLTensor {
 private:
  cudaGraphicsResource *cuda_graphics_resource_;

 public:
  CudaTensor(const GLTensorParam::ptr param) : GLTensor(param) {
    assert(param->device_id_ >= 0);
    assert(param->target_ == GL_RENDERBUFFER ||
           param->target_ == GL_TEXTURE_2D);
    checkCudaErrors(cudaGLSetGLDevice(param->device_id_));
    checkCudaErrors(cudaGraphicsGLRegisterImage(
        &cuda_graphics_resource_, image_, param->target_,
        cudaGraphicsRegisterFlagsReadOnly));
    checkCudaErrors(cudaMalloc(&data_ptr_, size_));
    tensor_ = torch::autograd::make_variable(at::native::from_blob(
        data_ptr_, {height_, width_, 4}, [](void *) {},
        at::device(at::kCUDA).dtype(at::ScalarType::Byte)));
  }

  virtual ~CudaTensor() {
    if (cuda_graphics_resource_) {
      checkCudaErrors(cudaGraphicsUnregisterResource(cuda_graphics_resource_));
    }

    if (data_ptr_) {
      checkCudaErrors(cudaFree(data_ptr_));
    }
    cudaDeviceReset();
  }

  virtual void *Data() const { return nullptr; }

 protected:
  void Update() {
    checkCudaErrors(cudaGraphicsMapResources(1, &cuda_graphics_resource_, 0));
    cudaArray *array = nullptr;
    checkCudaErrors(cudaGraphicsSubResourceGetMappedArray(
        &array, cuda_graphics_resource_, 0, 0));
    checkCudaErrors(cudaMemcpyFromArray(data_ptr_, array, 0, 0, size_,
                                        cudaMemcpyDeviceToDevice));
    checkCudaErrors(cudaGraphicsUnmapResources(1, &cuda_graphics_resource_, 0));
  }
};

class Cpu2CudaTensor : public GLTensor {
 public:
  Cpu2CudaTensor(const GLTensorParam::ptr param) : GLTensor(param) {
    assert(param->device_id_ >= 0);
    checkCudaErrors(cudaGLSetGLDevice(param->device_id_));
    checkCudaErrors(
        cudaHostAlloc(&data_ptr_, size_, cudaHostAllocWriteCombined));
    void *devie_data_ptr;
    checkCudaErrors(cudaHostGetDevicePointer(&devie_data_ptr, data_ptr_, 0));
    tensor_ = torch::autograd::make_variable(at::native::from_blob(
        devie_data_ptr, {height_, width_, 4}, [](void *) {},
        at::device(at::kCUDA).dtype(at::ScalarType::Byte)));
  }

  ~Cpu2CudaTensor() {
    if (data_ptr_) {
      checkCudaErrors(cudaFreeHost(data_ptr_));
    }
    cudaDeviceReset();
  }

 protected:
  void Update() {}
};

class CpuTensor : public GLTensor {
 private:
  void *host_data_ptr_;

 public:
  CpuTensor(const GLTensorParam::ptr param) : GLTensor(param) {
    data_ptr_ = malloc(size_);
    tensor_ = torch::autograd::make_variable(at::native::from_blob(
        data_ptr_, {height_, width_, 4}, [](void *) {},
        at::device(at::kCPU).dtype(at::ScalarType::Byte)));
  }

  ~CpuTensor() {
    if (data_ptr_) {
      free(data_ptr_);
    }
  }

 protected:
  void Update() {}
};

GLTensor *GLTensor::CreateCudaTensor(const GLTensorParam::ptr param) {
  return new CudaTensor(param);
}

GLTensor *GLTensor::CreateCpu2CudaTensor(const GLTensorParam::ptr param) {
  return new Cpu2CudaTensor(param);
}

GLTensor *GLTensor::CreateCpuTensor(const GLTensorParam::ptr param) {
  return new CpuTensor(param);
}

}  // namespace gltensor

#pragma  once
#include <memory>

#include <gl_tensor_param.h>
#include <torch/torch.h>

namespace gltensor {

class GLTensor {
 public:
  static GLTensor* CreateCudaTensor(const GLTensorParam::ptr param);
  static GLTensor* CreateCpu2CudaTensor(const GLTensorParam::ptr param);
  static GLTensor* CreateCpuTensor(const GLTensorParam::ptr param);

  virtual ~GLTensor();

  virtual at::Tensor Tensor();
  virtual void* Data() const { return data_ptr_; }

 protected:
  GLTensor(const GLTensorParam::ptr param);
  unsigned int image_;
  unsigned int width_;
  unsigned int height_;
  unsigned int size_;
  void* data_ptr_;
  at::Tensor tensor_;

  virtual void Update() = 0;
};

}  // namespace gltensor


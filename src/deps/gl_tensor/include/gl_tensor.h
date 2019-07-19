#pragma once
#include <memory>

#include <gl_tensor_param.h>
#include <torch/extension.h>

namespace gltensor {

class GLTensor {
 public:
  static GLTensor* CreateCudaTensor(const GLTensorParam::ptr param);

  virtual ~GLTensor();

  virtual at::Tensor Tensor();
  virtual void release(){};

 protected:
  GLTensor(const GLTensorParam::ptr param);
  unsigned int image_;
  unsigned int width_;
  unsigned int height_;
  unsigned int size_;
  unsigned int channels_;
  void* data_ptr_;
  at::Tensor tensor_;
  at::ScalarType type_;

  virtual void Update() = 0;
};

}  // namespace gltensor


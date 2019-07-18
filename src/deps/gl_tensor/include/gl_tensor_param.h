#pragma once

#include <memory>

namespace gltensor {

struct GLTensorParam {
  typedef std::shared_ptr<GLTensorParam> ptr;

  int device_id_ = 0;
  unsigned int image_ = 0;
  unsigned int target_ = 0;
  unsigned int width_ = 0;
  unsigned int height_ = 0;
  unsigned int format_ = 0;
};

}

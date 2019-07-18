#ifndef HEADLESS_GL_CONTEXT_H_
#define HEADLESS_GL_CONTEXT_H_

#include <glad/glad.h>
#include <memory>

namespace gltensortest {

struct EGLData;
class HeadlessGLContext;

typedef std::shared_ptr<HeadlessGLContext> GLContextPtr;

class HeadlessGLContext
    : public std::enable_shared_from_this<HeadlessGLContext> {
 public:
  ~HeadlessGLContext();
  void Bind();
  void Unbind();
  int GetDeviceId() const { return device_id_; }
  void* GetDisplay() const;
  void* GetContext() const;

  static GLContextPtr Create(int device_id = 0);

 private:
  int device_id_;
  std::unique_ptr<EGLData> egl_data_;

  HeadlessGLContext(int device_id);
  void Init(int device_id);
};

}  // namespace gltensortest

#endif
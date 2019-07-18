
#include <headless_gl_context.h>
#include <test_utils.h>

#include <glad/glad_egl.h>
#include <stdlib.h>
#include <memory>

namespace gltensortest {

struct EGLData {
  EGLDisplay display = EGL_NO_DISPLAY;
  EGLContext ctx = EGL_NO_CONTEXT;
};

GLContextPtr HeadlessGLContext::Create(int device_id) {
  return GLContextPtr(new HeadlessGLContext(device_id));
};

static const EGLint configAttribs[] = {EGL_SURFACE_TYPE,
                                       EGL_PBUFFER_BIT,
                                       EGL_BLUE_SIZE,
                                       8,
                                       EGL_GREEN_SIZE,
                                       8,
                                       EGL_RED_SIZE,
                                       8,
                                       EGL_DEPTH_SIZE,
                                       8,
                                       EGL_RENDERABLE_TYPE,
                                       EGL_OPENGL_BIT,
                                       EGL_CONFORMANT,
                                       EGL_OPENGL_BIT,
                                       EGL_NONE};
const int kMaxDevices = 32;

HeadlessGLContext::HeadlessGLContext(int device_id) {
  if (!gladLoadEGL()) {
    throw std::runtime_error("Failed to load EGL");
  }
  egl_data_ = std::unique_ptr<EGLData>(new EGLData());
  Init(device_id);
}

HeadlessGLContext::~HeadlessGLContext() {
  if (egl_data_->display != EGL_NO_DISPLAY) {
    printf("Destroying GL context\n");
    if (egl_data_->ctx != EGL_NO_CONTEXT) {
      eglMakeCurrent(egl_data_->display, EGL_NO_SURFACE, EGL_NO_SURFACE,
                     egl_data_->ctx);
      eglDestroyContext(egl_data_->display, egl_data_->ctx);
    }
    eglTerminate(egl_data_->display);
  }
}

void HeadlessGLContext::Init(int device_id) {
  EGLDeviceEXT egl_devices[kMaxDevices];
  EGLint num_devices;

  PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT =
      (PFNEGLQUERYDEVICESEXTPROC)eglGetProcAddress("eglQueryDevicesEXT");

  eglQueryDevicesEXT(kMaxDevices, egl_devices, &num_devices);
  EGL_CHECK_ERROR;

  printf("[EGLContext] Detected %d devices\n", num_devices);

  PFNEGLQUERYDEVICEATTRIBEXTPROC eglQueryDeviceAttribEXT =
      reinterpret_cast<PFNEGLQUERYDEVICEATTRIBEXTPROC>(
          eglGetProcAddress("eglQueryDeviceAttribEXT"));

  int egl_dev_id = 0;
  for (; egl_dev_id < num_devices; ++egl_dev_id) {
    EGLAttrib cuda_dev_number;
    if (eglQueryDeviceAttribEXT(egl_devices[egl_dev_id], EGL_CUDA_DEVICE_NV,
                                &cuda_dev_number) == EGL_TRUE &&
        cuda_dev_number == device_id) {
      break;
    }
  }

  if (egl_dev_id >= num_devices) {
    device_id_ = -1;
    printf("[EGLContext] Failed to find an EGL device for cuda device #%d\n",
           device_id);
    exit(1);
  }

  printf("[EGLContext] Selecting device %d\n", egl_dev_id);
  device_id_ = egl_dev_id;

  PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT =
      (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress(
          "eglGetPlatformDisplayEXT");
  EGL_CHECK_ERROR;

  egl_data_->display = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT,
                                                egl_devices[egl_dev_id], 0);
  EGL_CHECK_ERROR;

  EGLint major, minor;
  eglInitialize(egl_data_->display, &major, &minor);
  printf("[EGLContext] EGL Version: %d.%d\n", major, minor);
  EGL_CHECK_ERROR;

  EGLint num_configs;
  EGLConfig egl_cfg;

  eglChooseConfig(egl_data_->display, configAttribs, &egl_cfg, 1, &num_configs);
  EGL_CHECK_ERROR;

  // 4. Bind the API
  eglBindAPI(EGL_OPENGL_API);
  EGL_CHECK_ERROR;

  // 5. Create a context and make it current
  egl_data_->ctx =
      eglCreateContext(egl_data_->display, egl_cfg, EGL_NO_CONTEXT, NULL);
  Bind();

  if (!gladLoadGLLoader((GLADloadproc)eglGetProcAddress)) {
    throw std::runtime_error("Failed to load gl!");
  }
}

void HeadlessGLContext::Bind() {
  if (egl_data_->display != EGL_NO_DISPLAY &&
      egl_data_->ctx != EGL_NO_CONTEXT) {
    printf("[EGLContext] Bind EGL context\n");
    eglMakeCurrent(egl_data_->display, EGL_NO_SURFACE, EGL_NO_SURFACE,
                   egl_data_->ctx);
    EGL_CHECK_ERROR;
  }
}

void HeadlessGLContext::Unbind() {
  if (egl_data_->ctx != EGL_NO_CONTEXT &&
      eglGetCurrentContext() == egl_data_->ctx) {
    eglMakeCurrent(egl_data_->display, EGL_NO_SURFACE, EGL_NO_SURFACE,
                   EGL_NO_CONTEXT);
    EGL_CHECK_ERROR;
  }
}

EGLDisplay HeadlessGLContext::GetDisplay() const { return egl_data_->display; }

EGLContext HeadlessGLContext::GetContext() const { return egl_data_->ctx; }

}  // namespace gltensortest
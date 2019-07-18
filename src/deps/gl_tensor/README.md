# gl_tensor

## Purpose
This python plugin provides three ways to create Pytorch tensor directly from OpenGL rendering, on either a texture 2D or render buffer:
1. Plugin provides a host memory buffer; clients supplies the buffer to render engine; render engine fills the buffer (e.g. using glReadPixels) at each frame; plugin returns a CPU tensor. It's up to client to create a CUDA tensor, and push the CPU down to device. See test/test.py for sample code.
2. Plugin provides a host memory buffer, with a pinned counterpart on device; clients supplies the buffer to render engine; render engine fills the buffer (e.g. using glReadPixels) at each frame; plugin returns a CUDA tensor. In this case, all memory copying happens at C++ side, client doesn't have to do anything.
3. Render engines provides OpenGL image ID and target type; client forwards them to the plugin; plugin does the graphic-CUDA binding; after each frame, plugin returns a CUDA tensor. In this case, there is no device->host->device roundtrip memory copying.

## Benchmarks
Following results are generated on devfair with Nvidia QUADRO GP100 GPU.  
Image spec: 1024x1024 RGBA  
Method: render 360 frames, at each iteration generate a Pytorch CUDA tensor, record elapsed time, repeat three times, then take the average

* GPU->CPU->GPU Python: 0.521s, ~691 fps
* GPU->CPU->GPU C++: 0.176s, ~2041 fps
* GPU->GPU Graphic-CUDA Interop: 0.0163s, ~22086 fps

## Usage
* Render engine needs to provide `GLTensorParam` before rendering starts
  
        void* GetGLTensorParam() const {
          GLTensorParam *param = new GLTensorParam;
          param->device_id_ = gl_context_->GetDeviceId();
          param->image_ = image_;
          param->target_ = GL_TEXTURE_2D;
          param->width_ = kWidth;
          param->height_ = kHeight;
          param->format_ = GL_RGBA;
          return param;
        }
  
  then later on

        py::class_<GLTensorTest>(m, "GLTensorTest")
          .def(py::init<>())
          .def("GetGLTensorParam", &GLTensorTest::GetGLTensorParam,
               py::return_value_policy::take_ownership)
          .def("Frame", &GLTensorTest::Frame);

* Render engine needs to take `void *data` at each rendering call, and fill it with pixel data if `data != nullptr`. Note, this is not necessary if method #3 above is used 
* In client code

        # create rendering ending
        test = gl_tensor_test.GLTensorTest()
        # get tensor param
        param = test.GetGLTensorParam()
        # create a direct CUDA tensor
        gt = gl_tensor.CudaTensor(param)
        data = gt.Data()
        test.Frame(data)
        # t will be already a CUDA tensor
        t = gt.Tensor()
  
  More details can be found in test/test.py

## How to install
To install the plugin

    python setup.py build
    python setup.py install

To install the test

    cd test
    python setup.py build
    python setup.py install

Library paths to OpenGL and EGL may need to be added to `LD_LIBRARY_PATH` env variable, on devfair machines, it looks something like  
    
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu/nvidia-opengl:/usr/lib/x86_64-linux-gnu

## Limitations
1. The plugin depends on Pytorch 0.4.1. Pytorch 1.0 has different C++ API for creating tensors, so the plugin needs to be updated.
2. For direct CUDA tensor, plugin has to run the same thread as the renderer, since it needs to be using the same OpenGL context to do the graphic-CUDA binding.

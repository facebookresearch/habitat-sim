# gl_tensor

## Purpose
This python plugin creates a Pytorch tensor directly from OpenGL rendering, on either a texture 2D or render buffer.  The
rendering engine provides a OpenGL image ID and target type; client forwards them to the plugin; plugin does the graphic-CUDA binding; after each frame, plugin returns a CUDA tensor. There is no device->host->device roundtrip memory copying.

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
          auto param = std::make_shared<GLTensorParam>();
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
          .def("GetGLTensorParam", &GLTensorTest::GetGLTensorParam);

* In client code

        # create rendering ending
        test = gl_tensor_test.GLTensorTest()
        # get tensor param
        param = test.GetGLTensorParam()
        # create a direct CUDA tensor
        gt = gl_tensor.CudaTensor(param)
        test.Frame()
        # t will be already a CUDA tensor
        t = gt.Tensor()


## How to install
To install the plugin

    pip install .


Library paths to OpenGL and EGL may need to be added to `LD_LIBRARY_PATH` env variable, on devfair machines, it looks something like
    
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu/nvidia-opengl:/usr/lib/x86_64-linux-gnu

## Limitations
0. The plugin has to run the same thread as the renderer, since it needs to be using the same OpenGL context to do the graphic-CUDA binding.

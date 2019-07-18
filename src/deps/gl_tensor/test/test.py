import torch
import gl_tensor
import gl_tensor_test
import numpy as np
import matplotlib.image
import time

def test_cuda(test, frames):
  """Test OpenGL->CUDA direct memcpy
  Renders # of frames, then reports elapsed time
  """
  param = test.GetGLTensorParam()
  gt = gl_tensor.CudaTensor(param)
  data = gt.Data()

  # render one frame to warm up
  test.Frame(data)
  t = gt.Tensor()
  # make sure tensor is in correct size and already on device
  print("t - ", t.size(), t.is_cuda)

  start_t = time.process_time()
  for _ in range(0, frames):
    test.Frame(data)
    t = gt.Tensor()
    # if we want to output the image
    # img = t.cpu().numpy()
    # matplotlib.image.imsave(f'out/test{i:03}.png', img, vmin=0, vmax=255)
  elapsed_t = time.process_time() - start_t
  print("total time: ", elapsed_t)

def test_cpu(test, frames):
  """Test OpenGL->host->CUDA in Python route
  Renders # of frames, then reports elapsed time
  """
  param = test.GetGLTensorParam()
  gt = gl_tensor.CpuTensor(param)
  data = gt.Data()

  # render one frame to warm up
  test.Frame(data)
  t = gt.Tensor()
  # make sure tensor is in correct size and on host
  print("t - ", t.size(), t.is_cuda)
  # create a local tensor with pinned memory to device
  t = torch.tensor(data=gt.Tensor()).pin_memory().cuda()
  # now the tensor should be on device
  print("t - ", t.size(), t.is_cuda)

  start_t = time.process_time()
  for _ in range(0, frames):
    test.Frame(data)
    # copy the data back to device
    t.copy_(gt.Tensor())
  elapsed_t = time.process_time() - start_t
  print("total time: ", elapsed_t)


def test_cpu2cuda(test, frames):
  """Test OpenGL->host->CUDA in C++ route
  Renders # of frames, then reports elapsed time
  """
  param = test.GetGLTensorParam()
  gt = gl_tensor.Cpu2CudaTensor(param)
  data = gt.Data()

  # render one frame to warm up
  test.Frame(data)
  t = gt.Tensor()
  # make sure tensor is in correct size and already on device
  print("t - ", t.size(), t.is_cuda)

  start_t = time.process_time()
  for _ in range(0, frames):
    test.Frame(data)
    t = gt.Tensor()
  elapsed_t = time.process_time() - start_t
  print("total time: ", elapsed_t)

torch.cuda.set_device(0)
frames = 360

# print("run test_cpu...")
# test_cpu(gl_tensor_test.GLTensorTest(), frames)

# print("run test_cpu2cuda...")
# test_cpu2cuda(gl_tensor_test.GLTensorTest(), frames)

print("run test_cuda...")
test_cuda(gl_tensor_test.GLTensorTest(), frames)

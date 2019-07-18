from setuptools import setup
import os.path as osp

from torch.utils.cpp_extension import BuildExtension, CUDAExtension


base_dir = osp.abspath(osp.dirname(__file__))

setup(
    name="gl_tensor",
    ext_modules=[
        CUDAExtension(
            name="gl_tensor",
            sources=[
                osp.join(base_dir, s)
                for s in [
                    "glad/src/glad.c",
                    "glad/src/glad_egl.c",
                    "src/binding.cpp",
                    "src/gl_tensor.cpp",
                ]
            ],
            include_dirs=[
                osp.join(base_dir, s)
                for s in ["include", "include/cuda", "glad/include"]
            ],
            library_dirs=[
                "/usr/lib/x86_64-linux-gnu/",
                "/usr/lib/x86_64-linux-gnu/nvidia-opengl",
            ],
            libraries=["GL", "EGL"],
        )
    ],
    cmdclass={"build_ext": BuildExtension},
)

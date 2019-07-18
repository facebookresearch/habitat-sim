from setuptools import setup
from torch.utils.cpp_extension import CppExtension, BuildExtension

setup(
    name="gl_tensor_test",
    ext_modules=[
        CppExtension(
            name="gl_tensor_test",
            sources=[
                "../glad/src/glad.c",
                "../glad/src/glad_egl.c",
                "src/headless_gl_context.cpp",
                "src/test_utils.cpp",
                "src/main.cpp",
            ],
            include_dirs=["include", "../include", "../glad/include"],
            library_dirs=[
                "/usr/lib/x86_64-linux-gnu/",
                "/usr/lib/x86_64-linux-gnu/nvidia-opengl",
            ],
            libraries=["png", "z", "GL", "EGL"],
        )
    ],
    cmdclass={"build_ext": BuildExtension},
)

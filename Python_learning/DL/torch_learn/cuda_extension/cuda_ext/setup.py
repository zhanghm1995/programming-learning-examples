
from setuptools import setup
from torch.utils.cpp_extension import CUDAExtension, BuildExtension

setup(
    name='cuda_ext',
    version="1.2",
    packages=["cuda_ext"],
    ext_modules=[
        CUDAExtension(
            name="cuda_ext._cuda_ops",
            sources=['custom_cuda.cpp', 'custom_cuda_kernel.cu'],
        )
    ],
    cmdclass={'build_ext': BuildExtension},
)

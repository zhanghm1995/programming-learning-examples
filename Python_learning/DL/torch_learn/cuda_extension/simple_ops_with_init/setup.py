'''
Copyright (c) 2025 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2025-03-25 11:28:50
Email: haimingzhang@link.cuhk.edu.cn
Description: Without __init__.py, just import simple_ops and then using simple_ops.custom_add(a, b) to call the custom CUDA kernel.
直接使用 C++ 中通过 PYBIND11_MODULE 注册的函数
'''

from setuptools import setup
from torch.utils.cpp_extension import CUDAExtension, BuildExtension

setup(
    name='simple_ops_with_init',
    version="1.1",
    packages=["simple_ops_with_init"],
    ext_modules=[
        CUDAExtension(
            name="simple_ops_with_init._C",
            sources=['custom_cuda.cpp', 'custom_cuda_kernel.cu'],
        )
    ],
    cmdclass={'build_ext': BuildExtension},
)

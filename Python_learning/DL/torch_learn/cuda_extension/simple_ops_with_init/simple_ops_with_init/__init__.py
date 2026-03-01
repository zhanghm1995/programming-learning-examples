'''
Copyright (c) 2025 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2025-03-29 11:41:07
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import torch
from ._C import custom_add

class CustomCudaOperation:
    @staticmethod
    def add(input_tensor, value):
        """
        调用自定义 CUDA 扩展完成张量元素加操作

        Args:
            input_tensor (torch.Tensor): 输入张量
            value (float): 每个元素需要加上的值

        Returns:
            torch.Tensor: 输出张量
        """
        if not input_tensor.is_cuda:
            raise ValueError("Input tensor must be a CUDA tensor")
        return custom_add(input_tensor, value)


# __all__ = [  
#     'CustomCudaOperation',  
# ]

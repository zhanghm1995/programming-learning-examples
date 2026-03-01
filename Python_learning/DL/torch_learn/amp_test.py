'''
Copyright (c) 2024 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2024-10-23 10:46:17
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''
import os
import os.path as osp
import numpy as np
import pickle
import torch
from torch import nn  
from torch.cuda.amp import autocast


def example1():
    model = nn.Sequential(  
        nn.LayerNorm(10),   # 这个层通常会在fp32下执行  
        nn.Linear(10, 5),    # 假设这个层支持fp16  
        nn.Linear(5, 2),
        nn.Linear(2, 1),
    )  

    input = torch.randn(16, 10, dtype=torch.float32).cuda()  # 输入为fp16  
    input = torch.randn(16, 10, dtype=torch.float16).cuda()  # 输入为fp16  
    # input = torch.randn(16, 10, dtype=torch.float32).cuda() * 100000000000000  # 输入为fp16  

    # model = model.cuda().half()  # 将模型转换为fp16
    model = model.cuda()

    # x = model[0](input)  # LayerNorm 会输出fp32
    # print(x.dtype)
    # # 这里需要手动转换为fp16  
    # # x = x.half()  # 将fp32转换为fp16，以便接下来的操作可以继续使用fp16  
    # x1 = model[1](x)  # Linear 层继续执行，适合fp16
    # print(x1.dtype)

    # x = model[0](input)
    # x1 = model[1](x)

    # with autocast():  
    #     x = model[1](input)  # LayerNorm 会输出fp32  
    #     # 这里需要手动转换为fp16  
    #     # x = x.half()  # 将fp32转换为fp16，以便接下来的操作可以继续使用fp16  
    #     x1 = model[0](x)  # Linear 层继续执行，适合fp16
    #     print(x1.dtype)

    dtype = torch.get_autocast_gpu_dtype()
    print(dtype)

    print("input: ", input.dtype)
    with autocast():  
        x = model[0](input)  # LayerNorm 会输出fp32
        print(x.dtype)

        # 这里需要手动转换为fp16  
        # x = x.half()  # 将fp32转换为fp16，以便接下来的操作可以继续使用fp16  
        x1 = model[1](x)  # Linear 层继续执行，适合fp16
        print(x1.dtype)

        with autocast(enabled=False):
            x1 = x1.float()
            x2 = model[2](x1)
        print(x2.dtype)

        x3 = model[3](x2)
        print(x3.dtype)

    x2 = model[2](x1)  # Linear 层继续执行，适合fp16
    print(x2.dtype)


def example2():
    model = nn.Sequential(  
        nn.BatchNorm1d(10),   # 这个层通常会在fp32下执行
        nn.Linear(10, 5),
        nn.Linear(5, 2),
        nn.Linear(2, 1),
    )  

    input = torch.randn(16, 10, dtype=torch.float32).cuda()  # 输入为fp16  
    input = torch.randn(16, 10, dtype=torch.float16).cuda()  # 输入为fp16  
    # input = torch.randn(16, 10, dtype=torch.float32).cuda() * 100000000000000  # 输入为fp16  

    # model = model.cuda().half()  # 将模型转换为fp16
    model = model.cuda()
    
    print("input: ", input.dtype)
    with autocast():
        x = model[0](input)  # LayerNorm 会输出fp32
        # x = nn.ReLU()(x)
        print(x.dtype)

        # 这里需要手动转换为fp16  
        # x = x.half()  # 将fp32转换为fp16，以便接下来的操作可以继续使用fp16  
        x1 = model[1](x)  # Linear 层继续执行，适合fp16
        print(x1.dtype)


if __name__ == "__main__":
    example1()
    example2()
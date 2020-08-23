# -*- coding:utf8 -*-

import torch
import numpy as np

def construct_tensor():
    ## 创建一个直接包含具体数值的张量
    x = torch.tensor([5.5, 3])
    print(x, x.shape, type(x))
    
    ## 创建一个5x3的未初始化的张量,zeros才是全0的张量
    x = torch.empty(5, 3)
    print(x)

    ## 创建一个5x3全0张量
    x = torch.zeros(5, 3, dtype=torch.long)
    print(x)

    x = torch.rand(5, 3)
    print(x)

    x = x.new_ones(4, 3, dtype=torch.double)      # new_* methods take in sizes
    print(x)

    ## 继承张量的尺寸,使用新的指定数值类型
    x = torch.randn_like(x, dtype=torch.float)    # override dtype!
    print(x)

    x = torch.zeros_like(x)
    print(x)

    x = torch.ones_like(x)
    print(x)

    print(x.size())

def numpy_bridge():
    print("===============numpy_bridge=================")
    a = torch.ones(5)
    print(a, type(a))
    b = a.numpy().copy()
    print(b, type(b))
    b[0] = 10
    print(a)

    a = np.ones(5)
    b = torch.from_numpy(a)
    np.add(a, 1, out=a)
    print(a)
    print(b)

def operate_tensor():
    print("===============operate_tensor=================")
    x = torch.randn(5, 3)
    print(x)
    y = torch.randn(5, 3)
    print(y)

    ## 语法1,直接用+运算符
    print(x + y)
    ## 语法2, 调用torch.*函数
    print(torch.add(x, y))
    ## 语法3, 提供一个输出张量作为参数
    result = torch.empty(5, 3)
    torch.add(x, y, out=result)
    print(result)

    ## 语法4, in-place运算,即运算结果直接修改张量,带有_下划线结尾的函数都是in-place运算
    y.add_(x) # adds x to y
    print(y)

def resize_tensor():
    print("===============resize=================")
    ## 使用view函数可以用来修改张量的尺寸,注意view操作是共享内存的,即修改调整尺寸后的张量会影响原有张量
    x = torch.randn(4, 4)
    y = x.view(16)
    z = x.view(-1, 8)  # the size -1 is inferred from other dimensions
    print(x.size(), y.size(), z.size())

if __name__ == "__main__":
    construct_tensor()

    numpy_bridge()

    operate_tensor()

    resize_tensor()
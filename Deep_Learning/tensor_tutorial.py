# -*- coding:utf8 -*-

import torch

def construct_tensor():
    ## 创建一个直接包含具体数值的张量
    x = torch.tensor([5.5, 3])
    print(x, x.shape)
    
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

    print(x.size())

   


    

if __name__ == "__main__":
    construct_tensor()
import torch

def argmax_test():
    a = torch.randn(10,1)
    print(a.shape)
    print(a)
    print(torch.argmax(a))

    ## 找到前k个最大或最小数
    ## https://pytorch.org/docs/stable/generated/torch.topk.html
    _, top_idx = torch.topk(a[:,0], 3)
    print(top_idx)


def elements_test():
    ## 确定两个torch是否包含相同元素,可以broadcast,即判断一个元素是否在另一个torch中
    a = torch.Tensor([0, 1, 2, 4, 5])
    b = torch.Tensor([0])
    c = torch.eq(a, b)
    print(c)
    print(c.sum())

    

if __name__ == "__main__":
    argmax_test()
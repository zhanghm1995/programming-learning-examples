import torch
import numpy as np

def use_cuda():
    ## 判断是否能用GPU
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(device)
    
    ## 判断GPU显卡数量
    if torch.cuda.device_count() > 1:
        print("Let's use %d GPUs!" % (torch.cuda.device_count()))
    else:
        print("Only one GPU!")

def torch_min():
    x = torch.randn((2, 10))
    print(x)
    print(x.shape)
    y = torch.min(x, dim=1)
    print(y)
    print("===")
    votes_dist, _ = y
    print(votes_dist, votes_dist.shape)

def torch_argmax():
    print("Begin torch_argmax".center(40, '='))
    x = torch.randn((2, 10, 2)) * 10
    y = torch.argmax(x, 2)
    print(x)
    print(y, y.shape)
    print("End torch_argmax".center(40, '='))

if __name__ == "__main__":
    use_cuda()

    torch_min()

    torch_argmax()
    

    obb = np.zeros((6,8))
    print(obb)
    indices = np.arange(6)
    inds = [True, True, False, False, True, True]
    obb[inds, :] = 1
    print(obb)

    print(indices[inds])

    print("====")
    pc = np.random.randint(0,10,(10,3)) # [0,10)区间, shape为(4,3)的随机矩阵
    print(pc)
    num_sample = 4
    choices = np.random.choice(pc.shape[0], num_sample, replace=True)
    print(choices, choices.shape)
    pc = pc[choices]
    print(pc.shape)
    print(pc)
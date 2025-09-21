'''
Copyright (c) 2023 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2023-03-12 11:41:14
Email: haimingzhang@link.cuhk.edu.cn
Description: Learn the usage of DDP.
Reference: https://github.com/SingleZombie/DL-Demos/blob/master/dldemos/PyTorchDistributed/main.py
           https://zhuanlan.zhihu.com/p/350301395
'''

import os

import torch
import torch.distributed as dist
import torch.nn as nn
import torch.optim as optim
from torch.nn.parallel import DistributedDataParallel
from torch.utils.data import DataLoader, Dataset
from torch.utils.data.distributed import DistributedSampler


def setup():
    dist.init_process_group('nccl')


def cleanup():
    dist.destroy_process_group()


class ToyModel(nn.Module):

    def __init__(self) -> None:
        super().__init__()
        self.layer = nn.Linear(1, 1)

    def forward(self, x):
        return self.layer(x)


class MyDataset(Dataset):

    def __init__(self):
        super().__init__()
        self.data = torch.tensor([1, 2, 3, 4], dtype=torch.float32)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, index):
        return self.data[index:index + 1]


ckpt_path = 'tmp.pth'


def main():
    setup()
    rank = dist.get_rank()
    pid = os.getpid()
    num_replicas = dist.get_world_size()
    print(f'current pid: {pid}')
    print(f'Current rank {rank}')
    print(f'World size {num_replicas}')
    device_id = rank % torch.cuda.device_count()

    dataset = MyDataset()
    sampler = DistributedSampler(dataset)
    dataloader = DataLoader(dataset, batch_size=2, sampler=sampler)

    model = ToyModel().to(device_id)
    ddp_model = DistributedDataParallel(model, device_ids=[device_id])
    loss_fn = nn.MSELoss()
    optimizer = optim.SGD(ddp_model.parameters(), lr=0.001)

    if rank == 0:
        torch.save(ddp_model.state_dict(), ckpt_path)

    dist.barrier()

    map_location = {'cuda:0': f'cuda:{device_id}'}
    state_dict = torch.load(ckpt_path, map_location=map_location)
    print(f'rank {rank}: {state_dict}')
    ddp_model.load_state_dict(state_dict)

    for epoch in range(2):
        sampler.set_epoch(epoch)
        for x in dataloader:
            print(f'epoch {epoch}, rank {rank} data: {x}')
            x = x.to(device_id)
            y = ddp_model(x)
            optimizer.zero_grad()
            loss = loss_fn(x, y)
            loss.backward()
            optimizer.step()

    cleanup()


if __name__ == '__main__':
    main()
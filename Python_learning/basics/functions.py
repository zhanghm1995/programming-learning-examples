'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-10-14 00:22:17
Email: haimingzhang@link.cuhk.edu.cn
Description: Learn the functions usages in Python.
'''


class Dummy(object):
    def __init__(self, dataset="kitti", num_points=1024):
        args = locals()
        print(args, type(args))

        self.weight = 10.0
        print(self.__dict__)
        
        self.__dict__.update(args)
        print(self.__dict__)

        print(self.dataset, self.num_points)


from typing import Literal

def process_data(type: Literal['semantic'], data: str) -> str:
    if type == 'semantic':
        processed_data = f"Processing {type} data: {data}"
        return processed_data
    else:
        return "Invalid type specified."


def test_func():
    import numpy as np

    multi_adj_frame_id_cfg = (1, 2, 1)
    adj_id_list = list(range(*multi_adj_frame_id_cfg))
    print(adj_id_list)

    num_adj=len(range(*multi_adj_frame_id_cfg))
    print(num_adj)

    import torch
    input = torch.tensor([1, 2, 3, 4, 5, 6])
    input_a = input.view((3, 2))
    print(input_a)

    input_b = torch.split(input_a, 1, 1)
    print(input_b)


if __name__ == "__main__":
    test_func()
    exit()

    result = process_data('semantic', 'example data')
    result = process_data('depth', 'example data')
    print(result)

    # dummy = Dummy()
'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-06-29 14:31:10
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import numpy as np


class MyDataset(object):
    def __init__(self) -> None:
        super().__init__()
        self.data = list(range(10))

    def __iter__(self):
        for i, data in enumerate(self.data):
            yield i, data


dataset = MyDataset()
for i, data in dataset:
    print(i, data)
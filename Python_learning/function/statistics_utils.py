'''
Copyright (c) 2023 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2023-05-07 15:52:00
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import numpy as np


class AverageValueMeter(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.val = 0
        self.sum = 0
        self.count = 0.0

    def update(self, val, n=1):
        self.val = val
        self.sum += val * n
        self.count += n
    
    def add(self, sum, num):
        self.sum += sum
        self.count += num
    
    @property
    def avg(self):
        return self.sum / self.count


if __name__ == "__main__":
    batch1 = np.array([1, 2, 3, 4, 5])
    batch2 = np.array([1, 2, 3])

    meter = AverageValueMeter()
    meter.add(batch1.sum(), batch1.shape[0])
    meter.add(batch2.sum(), batch2.shape[0])
    print(meter.avg)

    actual_avg = (batch1.sum() + batch2.sum()) / (batch1.shape[0] + batch2.shape[0])
    print(actual_avg)

    meter2 = AverageValueMeter()
    meter.update(batch1.mean())
    meter.update(batch1.mean())
    print(meter.avg)

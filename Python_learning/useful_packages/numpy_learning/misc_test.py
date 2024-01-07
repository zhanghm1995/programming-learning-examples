'''
Copyright (c) 2023 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2023-11-10 08:55:28
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import numpy as np
import time


def test_padding_time():
    input = np.random.rand(2367, 2)

    start = time.time()
    for i in range(1000):
        pad_num = 5500 - input.shape[0]
        input_pad = np.pad(input, ((0, pad_num), (0, 0)), mode='constant')
    end = time.time()
    print("Time cost: {:.4f}".format(end - start))

    start = time.time()
    for i in range(1000):
        final_input = np.zeros((5500, 2))
        final_input[:input.shape[0], :] = input
    end = time.time()
    print("Time cost: {:.4f}".format(end - start))


if __name__ == "__main__":
    test_padding_time()
    print("Done!")
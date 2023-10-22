'''
Copyright (c) 2023 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2023-10-21 22:03:53
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import os


num_imgs = 10

# for i in range(num_imgs - 1):
#     for j in range(i + 1, num_imgs):
#         print(i, j)

# print('------------------')
# for i in range(num_imgs - 1, 0, -1):
#     for j in range(i - 1, -1, -1):
#         print(i, j)

print('------------------')
neighbor_length = 5
for i in range(num_imgs - 1):
    for j in range(i + 1, min(i + 1 + neighbor_length, num_imgs)):
        print(i, j)

print('------------------')
for i in range(num_imgs - 1, 0, -1):
    for j in range(i - 1, max(i - 1 - neighbor_length, -1), -1):
        print(i, j)
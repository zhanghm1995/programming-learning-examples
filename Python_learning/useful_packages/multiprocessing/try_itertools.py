'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-09-06 21:57:27
Email: haimingzhang@link.cuhk.edu.cn
Description: Have a try on itertools package.
'''

import itertools
from itertools import cycle
from tqdm import tqdm
import time


## Use the cycle function
# https://blog.csdn.net/u013066730/article/details/114278749
list_data = [1, 2, 3]

cycle_list = cycle(list_data)

count = 0
for i in tqdm(cycle_list):
    count += 1

    print(i)
    # time.sleep(1)
    if count >= 10:
        break

## Use the repeat function
repeated_list = itertools.repeat(list_data)
count = 0
for i in tqdm(repeated_list):
    count += 1

    print(i)
    # time.sleep(1)
    if count >= 10:
        break

print("Done")
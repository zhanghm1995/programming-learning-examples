'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-07-10 21:02:19
Email: haimingzhang@link.cuhk.edu.cn
Description: Use the time string in Python
'''

import time


def print_time_format():
    time_str = time.strftime("%Y-%m-%d %H:%M:%S")
    print(time_str)



if __name__ == "__main__":
    print_time_format()
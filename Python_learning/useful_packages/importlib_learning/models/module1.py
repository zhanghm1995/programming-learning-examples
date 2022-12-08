'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-11-30 15:32:23
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import os

def add(a, b):
    return a + b

class Model1(object):
    def __init__(self):
        pass

    def __call__(self, a, b):
        return a + b
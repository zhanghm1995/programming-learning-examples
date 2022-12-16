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


if __name__ == "__main__":
    dummy = Dummy()
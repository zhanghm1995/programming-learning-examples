#!/usr/bin/env python
# -*- coding:utf8 -*-
# 将常用命令行参数格式转换成VSCode需要的launch.json调试模式下的格式

"""
Usages: Put this file in your PATH path, and in any place you can execute the command:
convert_args_format_app.py "--data_root /home"

Note: You must specify your arguments with the quotation mark

Copyright (c) 2024 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2024-07-10 11:08:49
Email: haimingzhang@link.cuhk.edu.cn
Description: The Pretraining head.
"""

import sys


def chunks(xs, n):
    n = max(1, n)
    return [xs[i:i+n] for i in range(0, len(xs), n)]


def convert_arguments(input_argurments, need_multiple_lines=False, line_length=6):
    # input_argurments = "--name M003 --dataroot datasets/face/ --dataset_mode face --input_nc 15 --loadSize 512 --use_real_img --mode _M003_pose"
    origin_args_list = input_argurments.split()
    
    def join_args(args_list):
        args_str_list = [f"\"{arg}\"" for arg in args_list]

        args_formated = ", ".join(args_str_list)
        return args_formated

    if not need_multiple_lines:
        args_formated = join_args(origin_args_list)
    else:
        multi_args_list = chunks(origin_args_list, line_length)
        args_formated = []
        
        for sub_args_list in multi_args_list:
            sub_args_formated = join_args(sub_args_list)
            args_formated.append(sub_args_formated)
        
        args_formated = ",\n".join(args_formated)
        
    return args_formated
    

if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("Please add arguments needed to convet..., for example: python \"convert_args_format_app.py --name M003 --dataroot datasets/face/\"")
        exit(0)
    input_argument_str = sys.argv[1]
    print(f"You have input the arguments needed to convert: \n{input_argument_str}")
    result = convert_arguments(input_argument_str, need_multiple_lines=True, line_length=8)
    
    print("\nThe converted results is:")
    print(result)
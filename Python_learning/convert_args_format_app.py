#!/usr/bin/env python
# -*- coding:utf8 -*-
# 将常用命令行参数格式转换成VSCode需要的launch.json调试模式下的格式

"""
Usages: Put this file in your PATH path, and in any place you can execute the command:
convert_args_format_app.py "--data_root /home"

Note: You must specify your arguments with the quotation mark
"""

import sys


def convert_arguments(input_argurments):
    # input_argurments = "--name M003 --dataroot datasets/face/ --dataset_mode face --input_nc 15 --loadSize 512 --use_real_img --mode _M003_pose"
    args_list = input_argurments.split()
    args_list = [f"\"{arg}\"" for arg in args_list]

    args_formated = ", ".join(args_list)
    return args_formated
    

if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("Please add arguments needed to convet..., for example: python \"convert_args_format_app.py --name M003 --dataroot datasets/face/\"")
        exit(0)
    input_argument_str = sys.argv[1]
    print(f"You have input the arguments needed to convert: \n{input_argument_str}")
    result = convert_arguments(input_argument_str)
    
    print("\nThe converted results is")
    print(result)
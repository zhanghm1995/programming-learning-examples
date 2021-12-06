# -*- coding:utf8 -*-
# 将常用命令行参数格式转换成VSCode需要的launch.json调试模式下的格式
import sys


def convert_arguments(input_argurments):
    # input_argurments = "--name M003 --dataroot datasets/face/ --dataset_mode face --input_nc 15 --loadSize 512 --use_real_img --mode _M003_pose"
    args_list = input_argurments.split()
    args_list = [f"\"{arg}\"" for arg in args_list]

    args_formated = ", ".join(args_list)
    return args_formated
    

if __name__ == "__main__":
    input_argument_str = sys.argv[1]
    print(f"You have input the arguments needed to convert: \n{input_argument_str}")
    result = convert_arguments(input_argument_str)
    
    print("The converted results is")
    print(result)
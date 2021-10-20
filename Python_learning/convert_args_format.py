# -*- coding:utf8 -*-
# 将常用命令行参数格式转换成VSCode需要的launch.json调试模式下的格式

input_argurments = "./codes/main.py --exp_dir ./experiments_//001 --mode test --opt test.yml --gpu_ids 1"

args_list = input_argurments.split(" ")
args_list = [f"\"{arg}\"" for arg in args_list]

args_formated = ", ".join(args_list)
print(args_formated)
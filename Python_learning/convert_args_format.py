# -*- coding:utf8 -*-
# 将常用命令行参数格式转换成VSCode需要的launch.json调试模式下的格式

input_argurments = "--data_root dataset/talking_face_chinese/training2 --preprocessed_root dataset/chinese_preprocessed --ngpu 4"

args_list = input_argurments.split(" ")
args_list = [f"\"{arg}\"" for arg in args_list]

args_formated = ", ".join(args_list)
print(args_formated)
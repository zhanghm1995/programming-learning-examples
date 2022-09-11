# -*- coding:utf8 -*-

# Learn how to process the file path in Python

import os
import numpy as np


def process_dir():
    ## ----------------获取当前python文件所在文件夹绝对路径
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    print(type(BASE_DIR), BASE_DIR)
    print("=====", os.path.join(BASE_DIR, os.pardir))

    ## 创建文件夹
    # os.path.exists用来判断指定路径是否存在,包括文件或文件夹
    LOG_DIR = "test_dir"
    if not os.path.exists(LOG_DIR):
        os.mkdir(LOG_DIR)
    
    ## 执行系统命令删除文件夹
    if os.path.exists(LOG_DIR):
        print('Log folder %s already exists. Are you sure to overwrite? (Y/N)'%(LOG_DIR))
        c = input()
        if c == 'n' or c == 'N':
            print('Exiting..')
            exit()
        elif c == 'y' or c == 'Y':
            print('Remove the directory...')
            os.system('rm -r %s'%(LOG_DIR))
        else:
            print("You should enter Y or N to continue")
            exit()


if __name__=='__main__':
    process_dir()
# -*- coding:utf8 -*-

import os
import numpy as np

def process_dir():
    ## ----------------获取当前python文件所在文件夹绝对路径
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    print(type(BASE_DIR), BASE_DIR)
    print("=====", os.path.join(BASE_DIR, os.pardir))
    PARENT_DIR = os.path.abspath(os.path.join(BASE_DIR, os.pardir))

    ## 获取路径指定要素
    # basename表示路径字符串最后一个/后面的字符串内容
    print("------获取路径指定要素------")
    print(os.path.basename(BASE_DIR))
    print(os.path.basename(os.path.abspath(__file__)))

    ## 路径的拼接
    demo_dir = os.path.join(BASE_DIR, 'demo_files') 
    print(demo_dir)

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

    ## 列出指定目录下文件数量
    data_path = "/home/zhanghm/Temp"
    if os.path.exists(data_path):
        for x in os.listdir(data_path):
            # 此时x只是为目录下所有文件名而已,不包含路径
            print(os.path.basename(x))
    else:
        print("%s not exist"%data_path)

def pcoess_path_utils():
    ## ---------对路径进行拆分
    path_norm = os.path.normpath("/home/zhanghm/Deep_Learning/P2B_Project/votenet//sunrgbd/2.bin")
    print(path_norm, os.sep)
    path_split = path_norm.split(os.sep) # 得到字符串列表
    print(path_split)
    path_name = path_split[-1].replace(".bin", ".label")
    print(path_name)

def process_file_path():
    print("process_file_path")
    path = os.path.join("/home/zhanghm/Deep_Learning/P2B_Project/votenet/sunrgbd/sunrgbd_pc_bbox_votes_50k_v1_train/005051") + '_bbox.npy'
    print(path)

    ## 判断文件是否存在
    # os.path.expanduser('~')可以用来指代Linux下的home路径
    if os.path.exists(os.path.expanduser('~') + "/Utils/opencv"):
        print("File exist!")
    else:
        print("File not exist")

def process_misc():
    # os.getcwd()获取的是终端执行命令时的路径,并不一定是Python文件所在路径
    cwd = os.getcwd()
    print("BASE_DIR", cwd)

if __name__=='__main__':
    pcoess_path_utils()
    process_dir()
    exit()
    process_file_path()
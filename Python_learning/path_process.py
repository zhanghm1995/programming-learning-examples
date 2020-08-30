# -*- coding:utf8 -*-

import os
import numpy as np

def process_dir():
    ## 获取当前文件所在文件夹绝对路径
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    print(type(BASE_DIR), BASE_DIR)

    ## 获取路径指定要素
    # basename表示路径字符串最后一个/后面的字符串内容
    print("------获取路径指定要素------")
    print(os.path.basename(BASE_DIR))
    print(os.path.basename(os.path.abspath(__file__)))

    ## 路径的拼接
    demo_dir = os.path.join(BASE_DIR, 'demo_files') 
    print(demo_dir)

    ## 创建文件夹
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

    ## 列出指定目录下文件数量
    data_path = "/home/zhanghm/Temp"
    for x in os.listdir(data_path):
        # 此时x只是为目录下所有文件名而已,不包含路径
        print(os.path.basename(x))

def process_file_path():
    print("process_file_path")
    path = os.path.join("/home/z00520770/Deep_Learning/P2B_Project/votenet/sunrgbd/sunrgbd_pc_bbox_votes_50k_v1_train/005051") + '_bbox.npy'
    print(path)

if __name__=='__main__':
    process_dir()
    process_file_path()
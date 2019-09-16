# -*- coding:utf8 -*-  
"""
    Draw bar chart
"""
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

def plot_bar1():
    """
        draw esay bar chart
    """
    plt.figure(figsize=(8,6), dpi=80)

    plt.subplot(1,1,1)
    N = 6
    ## 每个柱形图对应值
    values = (25,32,34,20,41,50)

    index = np.arange(N)

    width = 0.35

    p2 = plt.bar(index, values, width, label="rainfall", color="g")

    # x轴标签
    plt.xlabel('Months',fontsize=13,fontweight='bold')
    plt.ylabel('rainfall (mm)',fontsize=13,fontweight='bold')
    plt.title('Monthly average rainfall', fontsize=13)
    plt.xticks(index, ('Jan', 'Fub', 'Mar', 'Apr', 'May', 'Jun'))
    plt.yticks(np.arange(0, 81, 10))

    # 图例
    # plt.legend(loc="upper right")
    plt.show()

def autolabels(rects):
    for rect in rects:
        height = rect.get_height()
        plt.text(rect.get_x()  + rect.get_width()/2.0-0.15, 1.01*height, '%s%%'%float(height), fontsize = 10)
def plot_bar2():
    n_groups = 5
    means_men = (20, 35, 30, 35, 27)
    std_men = (2,3,4,1,2)

    means_women = (25, 32, 34, 20, 25)
    std_women = (3, 5, 2, 3, 3)

    fig, ax = plt.subplots()
    plt.grid(linestyle = "--")
    index = np.arange(n_groups)
    bar_width = 0.35

    opacity = 0.4

    rects1 = ax.bar(index, means_men, bar_width, alpha=opacity, color='b', label='Men')
    autolabels(rects1)
    rects2 = ax.bar(index+bar_width, means_women, bar_width, alpha=opacity, color='r', label='Women')

    ax.set_xlabel('Group')
    ax.set_ylabel('Scores')
    ax.set_title('Scores by group and gender')
    ax.set_xticks(index + bar_width / 2)
    ax.set_xticklabels(('A','B','C','D','E'))
    ax.set_yticks(np.arange(0,101,10))
    ax.legend()
    
    plt.show()
    
def to_percent(temp, position):
    return '%1.0f'%(temp) + '%'

def plot_bar3():
    n_groups = 5

    is_sensors_fusion = False
    if is_sensors_fusion:
        means_MOTP = (85.45, 84.41, 83.51, 84.62, 89.74)
        means_MOTA = (39.07, 76.94, 71.01, 82.59, 76.84)
    else:
        means_MOTP = (63.15, 80.14, 76.31, 80.12, 51.41)
        means_MOTA = (23.27, 67.43, 67.42, 64.21, 36.34)


    fig, ax = plt.subplots()
    # plt.grid(linestyle = "--")
    index = np.arange(n_groups)
    bar_width = 0.35

    opacity = 0.7

    rects1 = ax.bar(index, means_MOTP, bar_width, alpha=opacity, color='b', label='MOTP')
    autolabels(rects1)
    rects2 = ax.bar(index+bar_width+0.04, means_MOTA, bar_width, alpha=opacity, color='r', label='MOTA')
    autolabels(rects2)
    ax.set_title('CLEAR MOT指标')
    ax.set_xticks(index + bar_width / 2)
    ax.set_xticklabels(('Dataset0000','Dataset0003','Dataset0005','Dataset0010','Dataset0018'))
    ax.set_yticks(np.arange(0,125,25))
    ax.yaxis.set_major_formatter(FuncFormatter(to_percent))
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), fancybox=True, ncol=5)
    # plt.savefig("clear_mot.svg",format='svg')
    plt.show()

def test():
    y_label = []
    for i in range(0,125,25):
        res = "%s%%"%i
        y_label.append(res)

if __name__ == "__main__":
    # plot_bar1()
    plot_bar3()
    # test()

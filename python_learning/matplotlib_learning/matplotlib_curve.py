"""
    Draw multiple curve plot
"""
import numpy as np
import matplotlib.pyplot as plt

x = np.array([3,5,7,9,11,13,15,17,19,21])
A = np.array([0.9708, 0.6429, 1, 0.8333, 0.8841, 0.5867, 0.9352, 0.8000, 0.9359, 0.9405])
B= np.array([0.9708, 0.6558, 1, 0.8095, 0.8913, 0.5950, 0.9352, 0.8000, 0.9359, 0.9419])
C=np.array([0.9657, 0.6688, 0.9855, 0.7881, 0.8667, 0.5952, 0.9361, 0.7848, 0.9244, 0.9221])
D=np.array([0.9664, 0.6701, 0.9884, 0.7929, 0.8790, 0.6072, 0.9352, 0.7920, 0.9170, 0.9254])

#label在图示(legend)中显示。若为数学公式，则最好在字符串前后添加"$"符号
#color：b:blue、g:green、r:red、c:cyan、m:magenta、y:yellow、k:black、w:white、、、
#线型：-    --     -.    :       , 
#marker：.    ,      o      v        <       *        +        1
plt.figure(figsize=(10,5))
plt.grid(linestyle = "--")            #设置背景网格线为虚线
ax = plt.gca()
ax.spines['top'].set_visible(False)   #去掉上边框
ax.spines['right'].set_visible(False) #去掉右边框

plt.plot(x,A,color="black",label="A algorithm",linewidth=1.5)
plt.plot(x,B,"k--",label="B algorithm",linewidth=1.5)
plt.plot(x,C,color="red",label="C algorithm",linewidth=1.5)
plt.plot(x,D,"r--",label="D algorithm",linewidth=1.5)

group_labels=['dataset1','dataset2','dataset3','dataset4','dataset5',' dataset6','dataset7','dataset8','dataset9','dataset10'] #x轴刻度的标识
plt.xticks(x,group_labels,fontsize=12,fontweight='bold')  #默认字体大小为10
plt.yticks(fontsize=12,fontweight='bold')
plt.title("example",fontsize=12,fontweight='bold')        #默认字体大小为12
plt.xlabel("Data sets",fontsize=13,fontweight='bold')
plt.ylabel("Accuracy",fontsize=13,fontweight='bold')
plt.xlim(3,21)                  #设置x轴的范围
#plt.ylim(0.5,1)

#plt.legend()                   #显示各曲线的图例
plt.legend(loc=0, numpoints=1)
leg = plt.gca().get_legend()
ltext  = leg.get_texts()
plt.setp(ltext, fontsize=12,fontweight='bold')  #设置图例字体的大小和粗细

plt.savefig('filename.svg',format='svg')    #建议保存为svg格式，再用inkscape转为矢量图emf后插入word中
plt.show()
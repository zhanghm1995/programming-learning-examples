import matplotlib.pyplot as plt  

# 数据准备  
data_ratios = [10, 25, 50, 100]  

# 三个方法在两个指标上的性能数据 (示例数据)  
method1_metric1 = [17.79, 22.76, 26.26, 31.37]  
method1_metric2 = [10.79, 19.40, 23.64, 27.89]  

method2_metric1 = [16.1, 22.34, 27.19, 34.03]  
method2_metric2 = [8.6, 18.28, 25.59, 30.12]  

method3_metric1 = [21.10, 27.21, 31.89, 39.36]  
method3_metric2 = [16.70, 26.59, 31.66, 34.67]  

# 创建两个子图  
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))  

# 第一个子图：指标1  
ax1.plot(data_ratios, method1_metric1, marker='o', label='Baseline', linestyle='-', color='b')  
ax1.plot(data_ratios, method2_metric1, marker='o', label='+UniPAD Pretrain', linestyle='-', color='g')  
ax1.plot(data_ratios, method3_metric1, marker='o', label='+Ours Pretrain', linestyle='-', color='r')  
ax1.set_xlabel('Data Proportion (%)')  
ax1.set_ylabel('NDS')  
ax1.set_title('Performance on NDS')  
ax1.legend()  
ax1.grid(True)  

# 第二个子图：指标2  
ax2.plot(data_ratios, method1_metric2, marker='x', label='Baseline', linestyle='--', color='b')  
ax2.plot(data_ratios, method2_metric2, marker='x', label='+UniPAD Pretrain', linestyle='--', color='g')  
ax2.plot(data_ratios, method3_metric2, marker='x', label='+Ours Pretrain', linestyle='--', color='r')  
ax2.set_xlabel('Data Proportion (%)')  
ax2.set_ylabel('mAP')  
ax2.set_title('Performance on mAP')  
ax2.legend()  
ax2.grid(True)  

# 调整布局以避免拥挤  
plt.tight_layout()  

# 显示图形  
plt.savefig("./data/data_efficiency2.png")
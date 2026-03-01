import matplotlib.pyplot as plt  

# 数据准备  
data_ratios = [10, 25, 50, 100]  

# 三个方法在两个指标上的性能数据 (示例数据)  
method1_metric1 = [0.6, 0.7, 0.8, 0.9]  
method1_metric2 = [0.5, 0.6, 0.75, 0.85]  

method2_metric1 = [0.65, 0.72, 0.82, 0.92]  
method2_metric2 = [0.55, 0.62, 0.78, 0.88]  

method3_metric1 = [0.7, 0.75, 0.85, 0.95]  
method3_metric2 = [0.6, 0.68, 0.8, 0.9]  

# 绘制折线图  
plt.figure(figsize=(10, 6))  

# 方法1  
plt.plot(data_ratios, method1_metric1, marker='o', label='Method 1 - Metric 1', linestyle='-')  
plt.plot(data_ratios, method1_metric2, marker='x', label='Method 1 - Metric 2', linestyle='--')  

# 方法2  
plt.plot(data_ratios, method2_metric1, marker='o', label='Method 2 - Metric 1', linestyle='-')  
plt.plot(data_ratios, method2_metric2, marker='x', label='Method 2 - Metric 2', linestyle='--')  

# 方法3  
plt.plot(data_ratios, method3_metric1, marker='o', label='Method 3 - Metric 1', linestyle='-')  
plt.plot(data_ratios, method3_metric2, marker='x', label='Method 3 - Metric 2', linestyle='--')  

# 设置图例、标签和标题  
plt.xlabel('Data Proportion (%)')  
plt.ylabel('Performance')  
plt.title('Performance Comparison of Three Methods')  
plt.legend()  
plt.grid(True)  

# 显示图形  
plt.savefig("./data/data_efficiency.png")
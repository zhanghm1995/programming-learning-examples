'''
Copyright (c) 2025 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2025-04-28 17:18:47
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''
# 五个人的名字  
persons = ["li", "xue", "ye", "luo", "zhang"]  

# 一周五天  
days = ["Mon", "Tue", "Wed", "Thu", "Fri"]  

# 示范：一周每天上午和下午的乘车人名单  
# 按天填写，例如: {'Mon': {'am': [...], 'pm': [...]}, ... }  
# ride_info = {  
#     "Mon": {"am": ["li", "zhang"],    "pm": ["li", "xue", "ye"]},  
#     "Tue": {"am": ["zhang", "ye"],    "pm": ["xue", "luo"]},  
#     "Wed": {"am": ["li", "luo"],      "pm": ["zhang", "ye", "luo"]},  
#     "Thu": {"am": ["xue", "ye", "luo"], "pm": ["li", "xue"]},  
#     "Fri": {"am": ["li", "zhang", "luo"], "pm": ["xue", "ye"]},  
# } 

all = ["li", "xue", "ye", "luo", "zhang"]

ride_info = {
    "Mon": {"am": all,    "pm": ["li", "xue", "luo", "zhang"] },  
    "Tue": {"am": all,      "pm": all},  
    "Wed": {"am": ["xue", "ye", "luo", "zhang"],      "pm": []}, 
    # "Thu": {"am": all, "pm": ['li', 'xue', 'luo', 'zhang']},
    # "Fri": {"am": ["li", "xue", "luo", "zhang"],      "pm": ['xue', 'luo', 'zhang']},  
}

days = ride_info.keys()

# 记录每个人的每天花费  
costs = {name: {day: 0 for day in days} for name in persons}  

# 记录每个人的总计  
total_cost = {name: 0 for name in persons}  

for day in days:  
    money = 30
    mornings = ride_info[day]["am"]  
    afternoons = ride_info[day]["pm"]  

    # 上午分摊  
    if mornings:  
        am_each = money / len(mornings)  
        for name in mornings:  
            costs[name][day] += am_each  
            total_cost[name] += am_each  

    # 下午分摊  
    if afternoons:  
        pm_each = money / len(afternoons)  
        for name in afternoons:  
            costs[name][day] += pm_each  
            total_cost[name] += pm_each  

# 打印结果  
print(f"{'Name':<8}", end='')  
for day in days:  
    print(f"{day:<12}", end='')  
print("Total")  
print("-" * (8 + 12 * 5 + 6))  

for name in persons:  
    print(f"{name:<8}", end='')  
    day_total = 0  
    for day in days:  
        day_cost = costs[name][day]  
        print(f"{day_cost:<12.2f}", end='')  
        day_total += day_cost  
    print(f"{total_cost[name]:.2f}")  

print()  
# 可选：打印上午、下午及每日总车费  
am_total = 0  
pm_total = 0  
for day in days:  
    am_total += 30  
    pm_total += 30  

print(f"\n上午总车费：{am_total} 元\n下午总车费：{pm_total} 元\n每日车费：60 元（总：{am_total+pm_total} 元）")  
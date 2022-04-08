'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-03-14 19:40:01
Email: haimingzhang@link.cuhk.edu.cn
Description: Example code for reading txt file
'''

def get_splited_filelists(filelists: list, split_length):
    data_splited = [filelists[i:i + split_length] for i in range(0, len(filelists), split_length)]
    return data_splited


# file_path = "/home/haimingzhang/Research/Programming/cv-fighter/HDTF_Dataset/HDTF_preprocessed/val.txt"
file_path = "bad_data3.txt"


lines = open(file_path).read().splitlines()

# print(len(lines), lines[46:48])

splited_list = get_splited_filelists(lines, 20)

print(len(splited_list))
choosed_list = splited_list[0]

output_str = ""
for item in choosed_list:
    output_str += f'\"{item}\"' + " "

output_str = f"FolderArray=({output_str.strip()})"
print(output_str)

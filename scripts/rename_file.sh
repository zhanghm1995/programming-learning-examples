#!/usr/bin/env bash
# Usage: Rename all files in a directory

dir_name="/media/zhanghm/Data/Datasets/KITTI/tracking/training/label_02"
# TODO: How to fill 0 before number

# 注意会直接替换当前文件
i=0
cd $dir_name
for file in `ls *.txt`
do
    echo $file
    newfile="$((i++)).txt"
    echo $newfile
    mv "$file" "$newfile"
done
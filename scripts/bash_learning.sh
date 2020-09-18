#!/bin/bash

echo "Hello world" "Hello world"
echo $0

echo "Start uploading $1..."

python data_uploader.py --dataset $1 --num_point 200
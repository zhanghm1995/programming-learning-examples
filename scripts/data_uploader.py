import os

print("Hello world from python")

import argparse

def parse_normal():
    """
    Example:
        python argument_parser.py --dataset=sunrgbd --num_point=10000
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', default='sunrgbd', help='Dataset: sunrgbd or scannet [default: sunrgbd]')
    parser.add_argument('--num_point', type=int, default=20000, help='Point Number [default: 20000]')
    FLAGS = parser.parse_args()
    print(FLAGS)
    print(type(FLAGS.dataset), type(FLAGS.num_point))


if __name__ == '__main__':
    parse_normal()

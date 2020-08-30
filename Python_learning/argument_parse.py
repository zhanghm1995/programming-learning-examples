# -*- coding:utf8 -*-

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

def parse_action():
    """
    命令行中只要出现--viz,--compute_median_size等参数则为true,否则为false，只为bool类型
    Example:
        python argument_parser.py --viz
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--viz', action='store_true', help='Run data visualization.')
    parser.add_argument('--compute_median_size', action='store_true', help='Compute median 3D bounding box sizes for each class.')
    parser.add_argument('--gen_v1_data', action='store_true', help='Generate V1 dataset.')
    parser.add_argument('--gen_v2_data', action='store_true', help='Generate V2 dataset.')
    args = parser.parse_args()

    print(args)
    print(type(args.viz))


if __name__=='__main__':
    parse_normal()
    # parse_action()
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
    命令行中只要出现--viz,--compute_median_size等参数则为true,否则为false, 只为bool类型
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


def modify_commandline_options(parser):
    # net structure and parameters
    parser.add_argument('--net_recon', type=str, default='resnet50', choices=['resnet18', 'resnet34', 'resnet50'], 
                        help='network structure')
    parser.add_argument('--init_path', type=str, default='checkpoints/init_model/resnet50-0676ba61.pth')
    parser.add_argument('--bfm_folder', type=str, default='BFM')
    parser.add_argument('--bfm_model', type=str, default='BFM_model_front.mat', help='bfm model')
    return parser


def parse_commandline_options():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser = modify_commandline_options(parser)
    opt, _ = parser.parse_known_args()  # parse again with new defaults

    message = ''
    message += '----------------- Options ---------------\n'
    for k, v in sorted(vars(opt).items()):
        comment = ''
        default = parser.get_default(k)
        if v != default:
            comment = '\t[default: %s]' % str(default)
        message += '{:>25}: {:<30}{}\n'.format(str(k), str(v), comment)
    message += '----------------- End -------------------'
    print(message)


if __name__=='__main__':
    # parse_normal()
    # parse_action()

    parse_commandline_options()

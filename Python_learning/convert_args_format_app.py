#!/usr/bin/env python
# -*- coding:utf8 -*-
# 将常用命令行参数格式转换成 VSCode launch.json 的 args 格式

"""
Usages:
convert_args_format_app.py "--data_root /home --use_real_img"

Note:
Arguments must be wrapped in quotation marks.

Author: Haiming Zhang
"""

import sys


def parse_arguments(arg_str):
    """
    将参数解析成 [(--arg, value), (--flag, True)] 形式
    """
    tokens = arg_str.split()
    parsed = []

    i = 0
    while i < len(tokens):
        token = tokens[i]

        if token.startswith("--"):
            # flag 参数（后面不是值）
            if i + 1 >= len(tokens) or tokens[i + 1].startswith("--"):
                parsed.append((token, "true"))
                i += 1
            else:
                parsed.append((token, tokens[i + 1]))
                i += 2
        else:
            i += 1

    return parsed


def format_launch_json_args(
    arg_pairs,
    max_line_length=80,
):
    """
    按 VSCode launch.json 格式输出，每行不超过 max_line_length
    """
    lines = []
    current_line = ""

    for key, value in arg_pairs:
        item = f'"{key}", "{value}"'

        if not current_line:
            current_line = item
        elif len(current_line) + len(item) + 2 <= max_line_length:
            current_line += ", " + item
        else:
            lines.append(current_line)
            current_line = item

    if current_line:
        lines.append(current_line)

    return ",\n".join(lines)


def convert_arguments(
    input_arguments,
    max_line_length=80,
):
    arg_pairs = parse_arguments(input_arguments)
    return format_launch_json_args(arg_pairs, max_line_length)


if __name__ == "__main__":
    if len(sys.argv) == 1:
        print(
            "Usage:\n"
            "python convert_args_format_app.py "
            '"--name M003 --dataroot datasets/face --use_real_img"'
        )
        sys.exit(0)

    input_argument_str = sys.argv[1]
    print("You have input the arguments:")
    print(input_argument_str)

    result = convert_arguments(
        input_argument_str,
        max_line_length=70,
    )

    print("\nConverted result (launch.json args format):\n")
    print(result)

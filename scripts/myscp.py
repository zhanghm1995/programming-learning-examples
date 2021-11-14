#!/usr/bin/env python
import os
import sys

"""Usages:
Place this file into /usr/bin or ~/.local/bin etc. path which was in your $PATH environment paths

myscp -r Amax:<path> <path>
myscp <path> A100:<path>
"""

remote_ip_dict = {'Amax': '10.20.18.21', 
                  'V100': '10.20.3.19', 
                  'A100': '120.204.84.133'}

user_name_dict = {'Amax': 'zhanghm', 
                  'V100': 'haimingzhang', 
                  'A100': 'dgxadmin'}


def build_remote_host_info():
    assert len(remote_ip_dict) == len(user_name_dict)

    remote_host_info_dict = dict()
    for key, value in user_name_dict.items():
        remote_host_info_dict[key] = f"{value}@{remote_ip_dict[key]}"
    return remote_host_info_dict


def run_copy(remote_host_info_dict, command_args_list):
    for server_name, info in remote_host_info_dict.items():
        command_args_list = command_args_list.replace(server_name, info)
    
    command_str = f"scp {command_args_list}"
    print(f"Execute command: \n {command_str}")
    os.system(command_str)


if __name__ == "__main__":
    remote_host_info_dict = build_remote_host_info()

    command_args_str = ' '.join(sys.argv[1:])
    run_copy(remote_host_info_dict, command_args_str)
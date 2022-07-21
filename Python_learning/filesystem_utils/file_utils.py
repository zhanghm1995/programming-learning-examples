'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-06-24 14:46:15
Email: haimingzhang@link.cuhk.edu.cn
Description: The file utility functions.
'''

import os
import os.path as osp


IMG_EXTENSIONS = [
    '.jpg', '.JPG', '.jpeg', '.JPEG',
    '.png', '.PNG', '.ppm', '.PPM', '.bmp', '.BMP', '.tiff'
]


def is_image_file(filename):
    """Check a file name is an image file or not.

    Args:
        filename (str|Path): The file name with extension.

    Returns:
        Bool: True if the file is an image file.
    """
    return any(filename.endswith(extension) for extension in IMG_EXTENSIONS)


def get_folder_list(input):
    """Get folder list when given a folder path or a text file containing folder paths.

    Args:
        input (str): folder root path or a text file containing folder paths.

    Raises:
        ValueError: _description_

    Returns:
        list: the list of folder names.
    """
    folder_list = None

    if osp.isdir(input):
        folder_list = [entry.name for entry in os.scandir(input) if entry.is_dir()]
        folder_list = sorted(folder_list)
    elif osp.isfile(input):
        folder_list = open(osp.join(input)).read().splitlines()
    else:
        raise ValueError(f"The input {input} is neither a folder nor a file.")
    return folder_list
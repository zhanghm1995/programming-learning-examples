'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-06-24 14:46:15
Email: haimingzhang@link.cuhk.edu.cn
Description: The file utility functions.
'''


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

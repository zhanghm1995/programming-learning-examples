'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-06-30 20:44:57
Email: haimingzhang@link.cuhk.edu.cn
Description: The utility functions for visualization
'''

import cv2
import numpy as np


def draw_points(image, points, add_text=True):
    """Draw points on the image.

    Args:
        image (np.ndarray): input image
        points (np.ndarray): (N, 2) array of points
        add_text (bool, optional): _description_. Defaults to True.

    """
    idx = -1
    for pt in points:
        idx += 1
        pt = (int(pt[0]), int(pt[1]))
        cv2.circle(image, pt, 5, (255, 0, 0), -1)

        if add_text:
            cv2.putText(image, str(idx), pt, cv2.FONT_HERSHEY_COMPLEX, 1.0, (0,0,255))


def draw_landmarks(img, landmark):
    """Draw landmarks in image, borrowed from Deep3DFaceRecon_Pytorch repo

    Args:
        img (_type_): _description_
        landmark (np.ndarray): (68, 2), u, v coordinates, but the v is flipped

    Returns:
        _type_: _description_
    """
    landmark = landmark
    lm_img = np.zeros([img.shape[0], img.shape[1], 3])
    lm_img[:] = img.astype(np.float32)
    landmark = np.round(landmark).astype(np.int32)

    for i in range(len(landmark)):
        for j in range(-1, 1):
            for k in range(-1, 1):
                if img.shape[0] - 1 - landmark[i, 1]+j > 0 and \
                        img.shape[0] - 1 - landmark[i, 1]+j < img.shape[0] and \
                        landmark[i, 0]+k > 0 and \
                        landmark[i, 0]+k < img.shape[1]:
                    lm_img[img.shape[0] - 1 - landmark[i, 1]+j, landmark[i, 0]+k,
                           :] = np.array([0, 0, 255])
    lm_img = lm_img.astype(np.uint8)
    return lm_img
'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-08-12 20:56:39
Email: haimingzhang@link.cuhk.edu.cn
Description: Some utility function related with geometry operations.
'''

import cv2
import numpy as np


def get_contour(im):
    contours, hierarchy = cv2.findContours(im.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    out = np.zeros_like(im)
    # On this output, draw all of the contours that we have detected
    # in white, and set the thickness to be 3 pixels
    # cv2.drawContours(out, contours, -1, 255, 3)

    out = cv2.fillPoly(out, contours, 255)

    return out


def get_masked_region(mask_img):
    gray = cv2.cvtColor(mask_img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    thresh = cv2.adaptiveThreshold(blur, 255, 1, 1, 11, 2)

    contour = get_contour(thresh)
    # contour = cv2.GaussianBlur(contour,(9, 9), 0)
    contour = cv2.erode(contour, np.ones((11, 11), np.uint8), iterations=1)
    return contour

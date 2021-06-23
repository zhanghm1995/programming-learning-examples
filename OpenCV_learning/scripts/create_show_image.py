import numpy as np
import sys
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2
width = 300
img = np.full((300,300,3), (0,255,0), np.uint8)
img2 = np.full((300,300,3), (0,0,255), np.uint8)
img_final = np.vstack((img, img2))
img_final2 = np.hstack((img, img2))
# img[:] = (255,0,0)
# img[:,0:width//2] = (255,0,0)      # (B, G, R)
# img[:,width//2:width] = (0,255,0)
# draw sth.
cv2.line(img_final2, (10,10), (250,50), (255, 255, 255), 6)
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.imshow('image',img_final2)
cv2.waitKey(0)
cv2.destroyAllWindows()


## Show text https://www.geeksforgeeks.org/python-opencv-cv2-puttext-method/
# Window name in which image is displayed 
window_name = 'Image'
  
# font 
font = cv2.FONT_HERSHEY_SIMPLEX 
  
# org 
org = (50, 50) 
  
# fontScale 
fontScale = 1
   
# Blue color in BGR 
color = (255, 0, 0) 
  
# Line thickness of 2 px 
thickness = 2
   
# Using cv2.putText() method 
image = cv2.putText(image, 'OpenCV', org, font,  
                   fontScale, color, thickness, cv2.LINE_AA) 
   
# Displaying the image 
cv2.imshow(window_name, image)
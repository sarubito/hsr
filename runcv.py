import cv2

import numpy as np

img=np.zeros([100,100])
print(type(img[0][0]))
a,mask = cv2.threshold(img,100,65535,cv2.THRESH_BINARY)
cv2.imshow("hehehe",mask)
cv2.waitKey(0)

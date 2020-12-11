import cv2
import numpy as np

img=np.zeros([100,100],dtype=np.uint16)
img[50:55]=60000
imgf= img.astype(np.float)+.001
print(type(imgf[0][0]))
a,b=cv2.threshold(imgf,10,65536,cv2.THRESH_BINARY)
cv2.imshow("a",img) 
cv2.waitKey(0)


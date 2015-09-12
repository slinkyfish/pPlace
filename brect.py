import numpy as np
import cv2

######### Import image, convert to grayscale  #########

im = cv2.imread('threshImg.jpg')
imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(imgray, 127, 255, 0)
im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

######### scroll through contours and find one of suitable size  #########

for count in range(len(contours)):
    area = cv2.contourArea(contours[count])
    print(area)                         #Display area of each contour found
    if(area >3000 and area < 4000):
      #print count
      chosen = count
      cnt = contours[count]

######### approximate contour as polygon with 10% deviation  #########

epsilon = 0.005*cv2.arcLength(cnt, True)            #([contour], [closed contour(T/F)])
approx = cv2.approxPolyDP(cnt, epsilon, True)

######### approximate contour as bounding rectangle  #########

rect = cv2.minAreaRect(cnt)
box =cv2.boxPoints(rect)
box = np.int0(box)
im = cv2.drawContours(im, [box], 0, (0, 0, 255), 2)

#im = cv2.drawContours(im, contours, chosen, (0, 255, 0), 1)

cv2.imshow("Keypoints", im)
cv2.waitKey(0)

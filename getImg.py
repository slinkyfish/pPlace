import cv2
import numpy as np
import picamera
import time
import serial
########### Initialise Serial Comms. with machine ###########
ser = serial.Serial('/dev/ttyUSB0', 115200)

###########     Position camera over backlight   ###########
#ser.write('G28\r\n')               ##comment out if camera is already in position
#ser.write('G1 Z25 F5000\r\n')
ser.write('G1 X145 Y90 F9000\r\n')

###########         Prepare Camera Settings       ###########
camera = picamera.PiCamera()
#camera.vflip = True
camera.exposure = _mode = 'manual'
camera.meter_mode = 'backlit'
camera.brightness = 60
camera.exposure = 100
camera.contrast = 100

#time.sleep(18)     ##comment out if camera is already in position

###########   Display Camera view and save image   ###########
camera.start_preview()
time.sleep(3)
camera.stop_preview()
camera.capture('img.jpg')

###########      Convert image to grayscale        ###########
im = cv2.imread("img.jpg")
gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
cv2.imwrite('grayIMG.jpg', gray)

###########  Convert grayscale to black and white    ###########
ret, threshImg = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
cv2.imwrite('threshImg.jpg', threshImg)

###########  Display output image    ###########
cv2.imshow("Output Image", threshImg)
cv2.waitKey(0)

import cv2
import numpy as np
import picamera
import time
import serial
import RPi.GPIO as GPIO
import math

component = {3780:'0805', 37200:'IC', 7080:'1206', 2022:'0603'}

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

backlight = 4
GPIO.setup(backlight, GPIO.OUT)

########### Initialise Serial Comms. with machine ###########
ser = serial.Serial('/dev/ttyUSB0', 115200)
ser.write("G21\r\n")        #Use mm
ser.write("G90\r\n")        #Use absolute coordinates
###########     Position camera over backlight   ###########
#ser.write('G28\r\n')               ##comment out if camera is already in position
ser.write('G1 Z25 F5000\r\n')
ser.write('G1 X145 Y90 F9000\r\n')

###########         Prepare Camera Settings       ###########
camera = picamera.PiCamera()
camera.led = False
#camera.vflip = True
camera.exposure = _mode = 'manual'
camera.meter_mode = 'backlit'
camera.brightness = 60
camera.exposure = 100
camera.contrast = 100

#time.sleep(18)     ##comment out if camera is already in position

###########   Display Camera view and save image   ###########
#camera.start_preview()
#time.sleep(3)
#camera.stop_preview()
GPIO.output(backlight, 1)
time.sleep(0.1)
camera.capture('img.jpg')
time.sleep(0.2)
GPIO.output(backlight, 0)

###########      Convert image to grayscale        ###########
im = cv2.imread("img.jpg")
gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
cv2.imwrite('grayIMG.jpg', gray)

###########  Convert grayscale to black and white    ###########
ret, threshImg = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY)   #([image], [threshold], [colour], [method])
cv2.imwrite('threshImg.jpg', threshImg)

###########  Display output image    ###########
#cv2.imshow("Output Image", threshImg)
#cv2.waitKey(0)


##################################################################

## Find part in image

##################################################################


######### Import image, convert to grayscale  #########

im = cv2.imread('threshImg.jpg')
imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(imgray, 127, 255, 0)
im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

######### scroll through contours and find one of suitable size  #########

for count in range(len(contours)):
    drawID = ''
    area = cv2.contourArea(contours[count])
    if(area > 200):
        print(area)                         #Display area of each contour found
        #drawID = str(area);                #Uncomment to draw area onto all objects found

    for i in range(len(component)):
        if(area < component.keys()[i]* 1.1 and area > component.keys()[i]*0.9):       #within suitable bounds (10%)
            print component[component.keys()[i]]
            drawID = component[component.keys()[i]]

    '''if(area >3000 and area < 4100):      
        drawID = '0805'
    if(area >33000 and area < 38400):
        drawID = 'IC'
    if(area >1750 and area < 2100):
        drawID = '0603'
    if(area >7000 and area < 7500):
        drawID = '1206'
        '''
    if(drawID):
        print "Model ID: " + str(drawID);
        print "Area: " + str(area)
        cnt = contours[count]
######### approximate contour as polygon with 10% deviation  #########

        epsilon = 0.01*cv2.arcLength(cnt, True)            #([contour], [closed contour(T/F)])
        approx = cv2.approxPolyDP(cnt, epsilon, True)

######### approximate contour as bounding rectangle  #########
        rect = cv2.minAreaRect(cnt)
        #print rect                  #holds ((x, y), (w, h), angle)
        
        xVal = int(rect[0][0])
        yVal = int(rect[0][1])
        wVal = int(rect[1][0])
        hVal = int(rect[1][1])
        error = abs((area-(wVal*hVal)))/(wVal*hVal) #amount of white as a fraction of the rectangle
        score = round((1-error)*100, 2)
        angleVal = int(rect[2])
        sinAngleVal = int(10*math.sin(rect[2]*3.14159/180))
        cosAngleVal = int(10*math.cos(rect[2]*3.14159/180))
        print "x: " + str(xVal)
        print "y: " + str(yVal)
        print "w: " + str(wVal)
        print "h: " + str(hVal)
        print "angle: " + str(angleVal)
        #print "10sinAngle: " + str(sinAngleVal)
        #print "10cosAngle: " + str(cosAngleVal)
        #print "Error: " + str(error)
        print "Score: " + str(score)
        print "\n"
        box =cv2.boxPoints(rect)
        box = np.int0(box)
        im = cv2.drawContours(im, [box], 0, (0, 0, 255), 2)
        cv2.putText(im, ('ID: ' + drawID), (xVal+2, yVal-2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 0))
        cv2.putText(im, ('Score: ' + str(score)), (xVal+2, yVal+12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 0))

       # im = cv2.circle(im, (xVal, yVal), 20, (0, 255, 0))
        im = cv2.line(im, (xVal-sinAngleVal, yVal-cosAngleVal), (xVal+sinAngleVal, yVal+cosAngleVal), (0, 0, 255))
        im = cv2.line(im, (xVal-cosAngleVal, yVal+sinAngleVal), (xVal+cosAngleVal, yVal-sinAngleVal), (0, 0, 255))
        im = cv2.line(im, (xVal-10, yVal), (xVal+10, yVal), (0, 255, 0))
        im = cv2.line(im, (xVal, yVal-10), (xVal, yVal+10), (0, 255, 0))
#im = cv2.drawContours(im, contours, chosen, (0, 255, 0), 1)

cv2.imshow("Keypoints", im)
cv2.waitKey(0)

cv2.imwrite('FoundImg.jpg', im)
GPIO.cleanup()

def boundaries(checkArea):
    if(checkArea > check*0.9 and checkArea < check*1.1):
        print "here"




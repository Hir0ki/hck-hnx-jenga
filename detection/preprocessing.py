import numpy as np
import cv2 



def get_roi(img):
    #rectangle 
    point1 = (200, 0 )
    point2 = (400, 500 )
    
    y = point1[1]
    x = point1[0] 
    w = point2[0] - point1[0] 
    h = point2[1] - point1[1]
    #reg_im = cv2.rectangle(img, point1, point2, (255,0,0), 3 )

    roi = img[y:y+h, x:x+w] 
    cv2.imshow('roi', roi)
    return roi

def get_hue(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_split = cv2.split(hsv)
    hue = hsv_split[0]
    return hue

def filtering(img):
    #blur 
    return cv2.medianBlur( img, 3)



def threshholing(img):
    #binary
    bin_image = cv2.inRange(img, 65, 100 )
    return bin_image

def denosing(img):

    #morpholigical stuff
    kernel = np.ones((3,3),np.uint8)
    opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    return opening

def do_preprossing(img):
    
    hue =  get_hue(img)

    median = filtering(hue)

    bin_img = threshholing(median)

    opening = denosing(bin_img)

    cv2.imshow('mask', bin_img)
    cv2.imshow('reg', opening)

    return opening

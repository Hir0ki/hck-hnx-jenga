import numpy as np 
import cv2


def getLines(src):
    laplace = cv2.Laplacian(src,cv2.CV_8UC1) #was CV_64F
    cv2.imshow("laplace", laplace)
    return laplace

def objExtraction(src):
    contours,_ = cv2.findContours(src,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    for cont in contours:
        x,y,w,h = cv2.boundingRect(cont)
        cv2.rectangle(src,(x,y),(x+w,y+h),(0,0,255),2)
    cv2.imshow("conturen",src)
    return contours

def create_bolb_detectoer():
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 90
    params.maxArea = 500
    #params.minThreshold = 10
    #params.maxThreshold = 255
    params.filterByConvexity = True
    params.minConvexity = 0.87  
    params.filterByColor = True
    params.blobColor  = 255
    return cv2.SimpleBlobDetector_create(params)

def blobDetector(src, roi, bin_img):
    # obsolete
    detector = create_bolb_detectoer()
    keypoints = detector.detect(bin_img)
    im_with_keypoints = cv2.drawKeypoints(src, keypoints, np.zeros([]), (0,0,255), 
    cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #print(keypoints)
    cv2.imshow("Keypointsimg", im_with_keypoints)
    return keypoints


def do_obj_detection(src, roi, bin_img):
    #laplace = getLines(src)
    #contour_img = objExtraction(laplace)
    blobimg =  blobDetector(src, roi, bin_img)
    return blobimg
import numpy as np 
import cv2


def objExtraction(src):
    contours,_ = cv2.findContours(src,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    cont_img = cv2.drawContours(src, contours, -1, (0,0,255))
    print(contours)
    cv2.imshow("objekte", cont_img)
    return contours

def do_obj_detection(src):
    laplace = cv2.Laplacian(src,3)
    cv2.imshow("conturen", laplace)
    #contours = objExtraction(laplace)
    return laplace
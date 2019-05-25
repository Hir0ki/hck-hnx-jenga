import numpy as np
import cv2


def translateMatrix(Keypoints):
    Matrix = []
    for i in Keypoints:
        Matrix.append(list(i.pt))
    npMatrix = np.asarray(Matrix)
    return npMatrix

def sortPoints(Matrix):
    sorted_raw = np.sort(Matrix)
    




def getTowerMatrix(Keypoints):
    # sort point coordinates for x and y

    npMatrix = translateMatrix(Keypoints)

    npMatrixSorted = sortPoints(npMatrix)

    return TowerMatrix
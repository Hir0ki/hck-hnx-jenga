import numpy as np
import cv2


def translateMatrix(Keypoints):
    Matrix = []
    for i in Keypoints:
        Matrix.append(list(i.pt))
    npMatrix = np.asarray(Matrix)
    return npMatrix

def sortPoints(Matrix):
    rows = []
    if Matrix != []:
        mat_sy = np.sort(Matrix, 1)

        i = 0 

        while i <= len(mat_sy):
            i += 3
            try:
                rows.append([list(mat_sy[i]), list(mat_sy[i+1]), list(mat_sy[i+2])])
            except IndexError:
                try: 
                    rows.append([list(mat_sy[i]), list(mat_sy[i+1])])
                except IndexError:
                    try:
                        rows.append([list(mat_sy[i])])
                    except IndexError:
                        pass
                        #print('rowng')
           

        return rows

def getTowerMatrix(Keypoints):
    # sort point coordinates for x and y

    npMatrix = translateMatrix(Keypoints)

    rows = sortPoints(npMatrix)

    if rows != None:

        return rows


class TowerMatrix():
    def getRemaining(self):
        # return complete matrix
        pass

    def getRowStatus(self, RowID):
        # belegungsplan
        pass
    
    def getNextMatrix(self):
        # neues bild, thread starten und x-mal laufen lassen, dann stop bis nächster aufruf
        pass


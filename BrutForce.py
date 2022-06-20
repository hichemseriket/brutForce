import cv2
import numpy as np
import datetime
import random
from random import randint


class BruteForcePolycubes():

    def __init__(self):
        self.polycubesTable = []
        self.explorationSpace = []

    # Fonction pour créer un espace d'exploration correspondant à un cube
    def CreateExplorationSpace(self, sizeVoxelsCube):
        positionsCube = np.array(np.meshgrid(np.arange(sizeVoxelsCube[0]), np.arange(sizeVoxelsCube[1]), np.arange(sizeVoxelsCube[2]), indexing='ij'))
        self.explorationSpace = np.transpose([positionsCube[0].flatten(), positionsCube[1].flatten(), positionsCube[2].flatten()])

    # Ajouter un polycube à la Polycubes table en incrémentant l'index
    def AddPolycubeToPolycubesTable(self, polycubeToAdd):
        maxIndex = self.polycubesTable[:, 3].max()
        print("maIndex", int(maxIndex))
        indexToAdd = 1 + maxIndex + np.zeros(shape=len(polycubeToAdd))
        self.polycubesTable = np.concatenate((self.polycubesTable, np.concatenate((polycubeToAdd, indexToAdd[:, None]), axis=1)), axis=0)

    # Fonction pour translater un polycube en x, y et z.
    def TranslatePolycube(self, indexPolycubeToTranslate, tx=0, ty=0, tz=0):
        self.polycubesTable[self.polycubesTable[:, 3] == indexPolycubeToTranslate] = self.polycubesTable[self.polycubesTable[:,3] == indexPolycubeToTranslate] + np.array([tx, ty, tz, 0])

    # Fonction pour rotater un polycube en donnant son index de rotation de 0 à 23 compris
    def RotatePolycube(self, indexPolycubeToRotate, indexRotation=0):
        polycubeToRotate = self.polycubesTable[self.polycubesTable[:, 3] == indexPolycubeToRotate]
        u, v, w, i = polycubeToRotate[:, 0], polycubeToRotate[:, 1], polycubeToRotate[:, 2], polycubeToRotate[:, 3]
        if indexRotation == 0:
            u, v, w = u, v, w
        elif indexRotation == 1:
            u, v, w = u, -v, -w
        elif indexRotation == 2:
            u, v, w = u, w, -v
        elif indexRotation == 3:
            u, v, w = u, -w, v
        elif indexRotation == 4:
            u, v, w = -u, v, -w
        elif indexRotation == 5:
            u, v, w = -u, -v, w
        elif indexRotation == 6:
            u, v, w = -u, w, v
        elif indexRotation == 7:
            u, v, w = -u, -w, -v
        elif indexRotation == 8:
            u, v, w = v, u, -w
        elif indexRotation == 9:
            u, v, w = v, -u, w
        elif indexRotation == 10:
            u, v, w = v, w, u
        elif indexRotation == 11:
            u, v, w = v, -w, -u
        elif indexRotation == 12:
            u, v, w = -v, u, w
        elif indexRotation == 13:
            u, v, w = -v, -u, -w
        elif indexRotation == 14:
            u, v, w = -v, w, -u
        elif indexRotation == 15:
            u, v, w = -v, -w, u
        elif indexRotation == 16:
            u, v, w = w, u, v
        elif indexRotation == 17:
            u, v, w = w, -u, -v
        elif indexRotation == 18:
            u, v, w = w, v, -u
        elif indexRotation == 19:
            u, v, w = w, -v, u
        elif indexRotation == 20:
            u, v, w = -w, u, -v
        elif indexRotation == 21:
            u, v, w = -w, -u, w
        elif indexRotation == 22:
            u, v, w = -w, v, u
        elif indexRotation == 23:
            u, v, w = -w, -v, -u
        self.polycubesTable[self.polycubesTable[:, 3] == indexPolycubeToRotate] = np.transpose([u, v, w, i])

    # Fonction pour pivoter un polycube en donnant son index de rotation de 0 à 23 compris et son index de voxel pivot
    # Le voxel pivot est le voxel dont la position est inchangée une fois la rotation effectuée, c'est le centre de rotation
    def PivotPolycube(self, indexPolycubeToPivot, indexRotation=0, indexVoxelPivot=0):
        polycubeToPivot = self.polycubesTable[self.polycubesTable[:, 3] == indexPolycubeToPivot]
        oldx, oldy, oldz = polycubeToPivot[indexVoxelPivot][:3]
        self.RotatePolycube(indexPolycubeToPivot, indexRotation)
        polycubeToPivot = self.polycubesTable[self.polycubesTable[:, 3] == indexPolycubeToPivot]
        newx, newy, newz = polycubeToPivot[indexVoxelPivot][:3]
        self.TranslatePolycube(indexPolycubeToPivot, oldx - newx, oldy - newy, oldz - newz)

    # Fonction pour calculer le nombre de voxels en dehors de l'espace de travail dans la configuration actuelle
    def CountVoxelsOut(self):
        nbVoxelsOut = 0
        for eachVoxel in self.polycubesTable[:, :3]:
            if not(np.any(np.all(eachVoxel == self.explorationSpace[:, :3], axis=1))):
                nbVoxelsOut = nbVoxelsOut + 1
        return nbVoxelsOut

    # Fonction pour calculer le nombre de voxels en collision dans la configuration actuelle
    def CountVoxelsInCollision(self):
        nbCollisions = 0
        for eachVoxel in self.polycubesTable[:, :3]:
            # First test if the voxel is inside the exploration space
            if np.any(np.all(eachVoxel == self.explorationSpace[:, :3], axis=1)):
                nbCollisions = nbCollisions + np.sum(np.all(eachVoxel == self.polycubesTable[:, :3], axis=1)) - 1
        return nbCollisions / 2

    # Fonction pour calculer le nombre de voxels remplis et uniques dans la configuration actuelle
    def CountVoxelsFilled(self):
        nbVoxelsFilled = 0
        for eachVoxel in self.polycubesTable[:, :3]:
            # First test if the voxel is inside the exploration space
            if np.any(np.all(eachVoxel == self.explorationSpace[:, :3], axis=1)):
                nbVoxelsFilled = nbVoxelsFilled + 1
        nbVoxelsFilled = nbVoxelsFilled - self.CountVoxelsInCollision()
        return nbVoxelsFilled

    # Fonction pour calculer le nombre de voxels laissés pour vide non remplis dans la configuration actuelle
    def CountVoxelsNotFilled(self):
        return len(self.explorationSpace) - self.CountVoxelsFilled()

    # Fonction pour calculer le pourcentage de voxels remplissant l'espace par rapport à la taille de l'espace à remplir dans le contexte actuel
    def CountPercentageSpaceFilled(self):
        return self.CountVoxelsFilled() / len(self.explorationSpace.shape)

    @staticmethod
    def Test():

        # Déclarer l'algorithme
        monAlgoBruteForce = BruteForcePolycubes()

        # Initialiser la seed
        random.seed(0)

        # Initialiser l'espace d'exploration
        monAlgoBruteForce.CreateExplorationSpace([3, 7, 12])

        # Déclarer la librairies de formes 3D à utiliser, ici trois formes sont utilisées mais il est possible d'en utiliser plus ou moins
        shape1 = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [2, 0, 1], [2, 0, 2], [3, 0, 2]])
        shape2 = np.array([[0, 0, 0], [0, 0, 1], [0, 1, 1], [0, 2, 1], [1, 2, 1]])
        shape3 = np.array([[0, 0, 0], [0, -1, 0], [0, -2, 0], [-1, -1, 0], [-2, -1, 0], [-2, -2, 0], [-2, -2, 1]])
        libraryShapes = [shape1, shape2, shape3]

        # Initialiser la table de polycubes avec un polycube choisi au hasard parmi la bibliothèque de formes et fixer son index à
        randomInit = random.randint(0, len(libraryShapes) - 1)
        initPolycube = libraryShapes[randomInit]
        indexToAdd = np.zeros(shape=len(libraryShapes[randomInit]))
        monAlgoBruteForce.polycubesTable = np.concatenate((initPolycube, indexToAdd[:, None]), axis=1)
        print("creation de la table ", monAlgoBruteForce.polycubesTable, "\n")

        monAlgoBruteForce.AddPolycubeToPolycubesTable(libraryShapes[2])
        print("ajout d'un polycube à la table, avant translation", monAlgoBruteForce.polycubesTable, "\n")

        monAlgoBruteForce.TranslatePolycube(1, 2, 0, 2)
        print("polycubesTable apres Translation du 2eme polycube avec 2,0,2", monAlgoBruteForce.polycubesTable, "\n")

        # monAlgoBruteForce.RotatePolycube(1, 12)
        # print(monAlgoBruteForce.polycubesTable, "\n")

        monAlgoBruteForce.PivotPolycube(0, 12, 3)
        print("polycubesTable apres pivot du polycube index 0 avec la rotation 12 (u, v, w = -v, u, w) selon le pivot 3", monAlgoBruteForce.polycubesTable, "\n")

        monAlgoBruteForce.TranslatePolycube(0, 0, -3, 2)
        print(monAlgoBruteForce.polycubesTable, "\n")

        monAlgoBruteForce.TranslatePolycube(0, 0, 2, 0)
        monAlgoBruteForce.TranslatePolycube(1, 0, 2, 0)

        print(monAlgoBruteForce.polycubesTable, "\n")

        print("C1 : ", monAlgoBruteForce.CountVoxelsOut())
        print("C2 : ", monAlgoBruteForce.CountVoxelsInCollision())
        print("C3 : ", monAlgoBruteForce.CountVoxelsFilled())
        print("C4 : ", monAlgoBruteForce.CountVoxelsNotFilled())
        print("C5 : ", monAlgoBruteForce.CountPercentageSpaceFilled())

        print(np.min(monAlgoBruteForce.explorationSpace, axis=0))
        print(np.max(monAlgoBruteForce.explorationSpace, axis=0))


BruteForcePolycubes.Test()

import cv2
import numpy as np
import datetime
import random


def souris(event, x, y, flags, param):
    global lo, hi, color
    if event == cv2.EVENT_LBUTTONDBLCLK:
        color = image[y, x][0]
        colors.append(image[y, x])
    if event == 1:
        setHighAndLow(image[y, x])
    elif event == 2:
        lo = lastLo
        hi = lastHi


def sendToCode(value):
    print('TODO send ', value)


def setHighAndLow(element):
    global minH, minS, minV, maxH, maxS, maxV, lo, hi, lastHi, lastLo
    lastHi = hi
    lastLo = lo
    if (element[0] < minH):
        minH = element[0]
    if (element[0] > maxH):
        maxH = element[0]
    if (element[1] < minS):
        minS = element[1]
    if (element[1] > maxS):
        maxS = element[1]
    if (element[2] < minV):
        minV = element[2]
    if (element[2] > maxV):
        maxV = element[2]
    lo = np.array([minH - 5, minS, minV])
    hi = np.array([maxH + 5, maxS, maxV])


# Permet de calibrer la couleur à suivre
minH = 255
minS = 255
minV = 255
maxH = 0
maxS = 0
maxV = 0
color = 100
lo = np.array([color - 5, 100, 50])
hi = np.array([color + 5, 255, 255])
lastLo = np.array([0, 0, 0])
lastHi = np.array([0, 0, 0])
color_info = (0, 0, 255)
cap = cv2.VideoCapture(0)
cv2.namedWindow('Camera')
cv2.setMouseCallback('Camera', souris)
print('config low : ', lo, ' high: ', hi)
colors = []
while True:
    ret, frame = cap.read()
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    image = cv2.blur(image, (5, 5))
    mask = cv2.inRange(image, lo, hi)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)
    image2 = cv2.bitwise_and(frame, frame, mask=mask)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
    cv2.imshow('Camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# permet de définir les valeurs min et max
print('fin de la définition de la couleur')

# run du jeu
# definition du point d'arrivée
width = cap.get(3)
height = cap.get(4)
date = datetime.datetime.now()
tour = 0


def runGame(tour):
    if (tour < 3):
        xa = random.randint(0, width)
        ya = random.randint(0, height)
        print(lo, hi, 'high,, low')
        lastDistance = 100000000
        while True:
            ret, frame = cap.read()
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            image = cv2.blur(image, (5, 5))
            mask = cv2.inRange(image, lo, hi)
            mask = cv2.erode(mask, None, iterations=1)
            mask = cv2.dilate(mask, None, iterations=1)
            frame = cv2.circle(frame, (xa, ya), radius=5, color=(0, 0, 255), thickness=-1)
            image2 = cv2.bitwise_and(frame, frame, mask=mask)
            elements = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            if len(elements) > 0:
                c = max(elements, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                if radius > 30:
                    print(x, y, 'x, y')
                    newDistance = ((((x - xa) ** 2) + ((y - ya) ** 2)) ** 0.5)
                    if int(newDistance) >= 10:
                        if (lastDistance > newDistance):
                            sendToCode('+')
                        else:
                            sendToCode('-')
                    else:
                        sendToCode('=')
                        break
                    lastDistance = newDistance
                    cv2.circle(frame, (int(x), int(y)), 5, color_info, 10)
            cv2.imshow('Camera', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        runGame(tour + 1)


runGame(tour)
cap.release()
cv2.destroyAllWindows()
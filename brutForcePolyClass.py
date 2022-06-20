import cv2
import numpy as np
import datetime
import random
from random import randint


class BruteForcePolycubes():

    def __init__(self):
        self.polycubesTable = []
        self.explorationSpace = []
        self.libraryShapes = []
        self.maxIndex = 0

    # Fonction pour créer un espace d'exploration correspondant à un cube
    def CreateExplorationSpace(self, sizeVoxelsCube):
        positionsCube = np.array(np.meshgrid(np.arange(sizeVoxelsCube[0]), np.arange(sizeVoxelsCube[1]), np.arange(sizeVoxelsCube[2]), indexing='ij'))
        self.explorationSpace = np.transpose([positionsCube[0].flatten(), positionsCube[1].flatten(), positionsCube[2].flatten()])

    def CreateShapesLibrary(self, myLibraryShapes):
        self.libraryShapes = myLibraryShapes

    def InitialisationFirstShape(self, indexFirstShape = -1):
        # Initialiser la table de polycubes avec un polycube choisi au hasard parmi la bibliothèque de formes et fixer son index à
        #If the first shape index is not given bgy the user, then choose one randomly
        if indexFirstShape == -1:
            indexFirstShape = random.randint(0, len(self.libraryShapes) - 1)
        initPolycube = self.libraryShapes[indexFirstShape]
        indexToAdd = np.zeros(shape=len(self.libraryShapes[indexFirstShape]))
        self.polycubesTable = np.concatenate((initPolycube, indexToAdd[:, None]), axis=1)
        print("creation de la table ", self.polycubesTable, "\n")

    # Ajouter un polycube à la Polycubes table en incrémentant l'index
    def AddPolycubeToPolycubesTable(self, polycubeToAdd):
        self.maxIndex = self.maxIndex + 1
        indexToAdd = self.maxIndex + np.zeros(shape=len(polycubeToAdd))
        self.polycubesTable = np.concatenate((self.polycubesTable, np.concatenate((polycubeToAdd, indexToAdd[:, None]), axis=1)), axis=0)

    # Ajouter un polycube à la Polycubes table en incrémentant l'index
    def AddPolycubeToPolycubesTableByIndex(self, indexPolycubeToAdd):
        self.AddPolycubeToPolycubesTable(self.libraryShapes[indexPolycubeToAdd])

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

    def Agent(self, indexShapeInShapeLibraryToAdd, indexRotation, tx, ty, tz):
        self.AddPolycubeToPolycubesTableByIndex(indexShapeInShapeLibraryToAdd)
        self.RotatePolycube(len(self.polycubesTable) - 1, indexRotation)
        self.TranslatePolycube(len(self.polycubesTable) - 1, tx, ty, tz)



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

        monAlgoBruteForce.CreateShapesLibrary(libraryShapes)

        monAlgoBruteForce.InitialisationFirstShape()

        #monAlgoBruteForce.AddPolycubeToPolycubesTable(libraryShapes[2])
        monAlgoBruteForce.AddPolycubeToPolycubesTableByIndex(2)
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

    @staticmethod
    def Test2():

        for j in range(100):

            # Déclarer l'algorithme
            monAlgoBruteForce = BruteForcePolycubes()

            # Initialiser la seed
            random.seed(j)

            # Initialiser l'espace d'exploration et calculer sa taille
            monAlgoBruteForce.CreateExplorationSpace([3, 7, 12])
            xmin, ymin, zmin = np.min(monAlgoBruteForce.explorationSpace, axis=0)
            xmax, ymax, zmax = np.max(monAlgoBruteForce.explorationSpace, axis=0)

            # Déclarer la librairies de formes 3D à utiliser, ici trois formes sont utilisées mais il est possible d'en utiliser plus ou moins
            shape1 = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [2, 0, 1], [2, 0, 2], [3, 0, 2]])
            shape2 = np.array([[0, 0, 0], [0, 0, 1], [0, 1, 1], [0, 2, 1], [1, 2, 1]])
            shape3 = np.array([[0, 0, 0], [0, -1, 0], [0, -2, 0], [-1, -1, 0], [-2, -1, 0], [-2, -2, 0], [-2, -2, 1]])
            libraryShapes = [shape1, shape2, shape3]
            monAlgoBruteForce.CreateShapesLibrary(libraryShapes)

            monAlgoBruteForce.InitialisationFirstShape()

            for i in range(20):
                randomIndexShapeToAdd = np.random.randint(0, len(libraryShapes) - 1)
                randomIndexRotation = np.random.randint(0, 23)   #ou 24 tu verras
                randomTx = np.random.randint(xmin, xmax)
                randomTy = np.random.randint(ymin, ymax)
                randomTz = np.random.randint(zmin, zmax)
                monAlgoBruteForce.Agent(randomIndexShapeToAdd, randomIndexRotation, randomTx, randomTy, randomTz)

            #print(monAlgoBruteForce.polycubesTable)

            print(j, "C1 : ", monAlgoBruteForce.CountVoxelsOut())
            print(j, "C2 : ", monAlgoBruteForce.CountVoxelsInCollision())
            #print("C3 : ", monAlgoBruteForce.CountVoxelsFilled())
            #print("C4 : ", monAlgoBruteForce.CountVoxelsNotFilled())
            #print("C5 : ", monAlgoBruteForce.CountPercentageSpaceFilled())


BruteForcePolycubes.Test2()
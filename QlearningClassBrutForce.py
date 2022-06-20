import cv2
import numpy as np
import datetime
import random
from random import randint


class BruteForcePolycubes():

    def __init__(self):
        # self.sizeVoxelsCube = 1
        self.polycubesTable = []
        self.explorationSpace = []

    # Fonction pour créer un espace d'exploration correspondant à un cube
    def CreateExplorationSpace(self, sizeVoxelsCube):
        positionsCube = np.array(
            np.meshgrid(np.arange(sizeVoxelsCube[0]), np.arange(sizeVoxelsCube[1]), np.arange(sizeVoxelsCube[2]),
                        indexing='ij'))
        self.explorationSpace = np.transpose(
            [positionsCube[0].flatten(), positionsCube[1].flatten(), positionsCube[2].flatten()])

        # reset function for the class

    def reset(self):
        self.polycubesTable = []
        self.explorationSpace = []

    # function to clear the table
    def clearTable(self):
        self.polycubesTable = []


    # Ajouter un polycube à la Polycubes table en incrémentant l'index
    def AddPolycubeToPolycubesTable(self, polycubeToAdd):
        maxIndex = self.polycubesTable[:, 3].max()
        indexToAdd = 1 + maxIndex + np.zeros(shape=len(polycubeToAdd))
        self.polycubesTable = np.concatenate(
            (self.polycubesTable, np.concatenate((polycubeToAdd, indexToAdd[:, None]), axis=1)), axis=0)

    # Fonction pour translater un polycube en x, y et z.
    def TranslatePolycube(self, indexPolycubeToTranslate, tx=0, ty=0, tz=0):
        self.polycubesTable[self.polycubesTable[:, 3] == indexPolycubeToTranslate] = self.polycubesTable[
                                                                                         self.polycubesTable[:,
                                                                                         3] == indexPolycubeToTranslate] + np.array(
            [tx, ty, tz, 0])

    # add reset function to CreateExplorationSpace


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
            if not (np.any(np.all(eachVoxel == self.explorationSpace[:, :3], axis=1))):
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

    # get neighbours of polycube
    def getNeighbours(self, polycube):
        neighbours = []
        for i in range(len(self.polycubesTable)):
            if np.any(np.all(self.polycubesTable[i][:3] == polycube[:3], axis=1)):
                neighbours.append(i)
        return neighbours

    # count number of DOFs
    def countDOFs(self):
        nbDOFs = 0
        for i in range(len(self.polycubesTable)):
            bloc = nbDOFs + len(self.getNeighbours(self.polycubesTable[i]))
            nbDOFs = 6 - bloc

        return nbDOFs

    # give reward if more than 2 neighbours
    def giveReward(self):
        nbNeighbours = 0
        for i in range(len(self.polycubesTable)):
            if len(self.getNeighbours(self.polycubesTable[i])) > 2:
                nbNeighbours = nbNeighbours + 1
        if nbNeighbours > 2:
            return 1
        else:
            return 0



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
        print(
            "polycubesTable apres pivot du polycube index 0 avec la rotation 12 (u, v, w = -v, u, w) selon le pivot 3",
            monAlgoBruteForce.polycubesTable, "\n")

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

    def isDone(self):
        pass

    def giveInfo(self):
        pass


BruteForcePolycubes.Test()
# class polycube and methode create polycube
# class polycube and methode create polycube
class Polycube:
    def __init__(self, index, polycube):
        self.index = index
        self.polycube = polycube

    # Fonction pour créer un polycube
    def createPolycube(self, sizeVoxelsCube):
        polycube = np.array(
            np.meshgrid(np.arange(sizeVoxelsCube[0]), np.arange(sizeVoxelsCube[1]), np.arange(sizeVoxelsCube[2]),
                        indexing='ij'))
        polycube = np.transpose([polycube[0].flatten(), polycube[1].flatten(), polycube[2].flatten()])
        return polycube

    # reset function for the polycube
    def resetPolycube(self):
        self.polycubesTable = []
        self.index = 0

    def GetIndex(self):
        return self.index

    def GetPolycube(self):
        return self.polycube


# exemples of basic qlearning

import numpy as np
import gym
import gym_minigrid
import time


def state_to_key(obs=(0, 0, 0)):
    return BruteForcePolycubes().AddPolycubeToPolycubesTable(obs)


def update_Q(Q, s, sp, a, r, done):
    if s not in Q:
        Q[s] = np.array([0., 0., 0., 0.])
    if sp not in Q:
        Q[sp] = np.array([0., 0., 0., 0.])

    ap = np.argmax(Q[sp])
    if not done:
        Q[s][a] = Q[s][a] + 0.01 * (r + 0.99 * Q[sp][ap] - Q[s][a])
    else:
        Q[s][a] = Q[s][a] + 0.01 * (r - Q[s][a])


def create_state_if_not_exist(Q, s):
    if s not in Q:
        Q[s] = np.array([0., 0., 0., 0.])


def main():
    Q = {}
    #instantiate the the class BruteForcePolycubes
    monAlgoBruteForce = BruteForcePolycubes()
    monPolycube = Polycube(0, 0)

    # le tuto use env grid
    # env = gym.make("MiniGrid-Empty-6x6-v0")
    # ill use my env
   # env = np.transpose([positionsCube[0].flatten(), positionsCube[1].flatten(), positionsCube[2].flatten()])
   #  env = monPolycube.createPolycube(sizeVoxelsCube=[4, 4, 4])
   #  env = monAlgoBruteForce.CreateExplorationSpace(sizeVoxelsCube=[4, 4, 4])
    env = monAlgoBruteForce

    eps = 0.01

    for epoch in range(10):

        s = env.clearTable()
        print("s au debut : ", s)
        print("type of s : ", type(s))
        s = state_to_key(s)
        print("s apres conversion state_key : ", s)
        done = False

        while not done:

            if np.random.rand() < eps:
                a = np.random.randint(0, 4)
                print("a debut du while: ", a)
            else:
                create_state_if_not_exist(Q, s)
                a = np.argmax(Q[s])


            sp, r, done, info = monAlgoBruteForce.PivotPolycube(a),monAlgoBruteForce.giveReward(),monAlgoBruteForce.isDone(),monAlgoBruteForce.giveInfo()
            print("sp dans les statements longs : ", sp)
            print("r dans les statements longs :  ", r)
            print("done dans les statements longs : ", done)
            print("info dans les statements longs : ", info)
            sp = state_to_key(sp)
            print("sp apres les statements longs : ", sp)

            update_Q(Q, s, sp, a, r, done)

            s = sp
            print("s apres update_Q : ", s)
        print("eps", eps)
        eps = max(0.1, eps * 0.99)

    for epoch in range(10):
        s = monAlgoBruteForce.reset()
        s = state_to_key(s)
        done = False

        while not done:
            create_state_if_not_exist(Q, s)
            print("create_state_if_not_exist : ", s)
            a = np.argmax(Q[s])
            print("a np.argMax : ", a)
            sp, r, done, info = monAlgoBruteForce.PivotPolycube(a),monAlgoBruteForce.giveReward(),monAlgoBruteForce.isDone(),monAlgoBruteForce.giveInfo()
            sp = state_to_key(sp)
            s = sp
            time.sleep(0.1)
        print("r", r)


if __name__ == "__main__":
    main()

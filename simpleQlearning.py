import numpy as np
from random import randint
import random
import datetime
import gym
import gym_minigrid
import time

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
        # self.polycubesTable = []
        self.explorationSpace = []

    # function to clear the table
    def clearTable(self):
        self.polycubesTable = []
        self.CreateExplorationSpace = []


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

    def isDone(self):
        pass

    def giveInfo(self):
        pass









class EnvGrid(object):
    """
        docstring forEnvGrid.
    """
    monAlgoBruteForce = BruteForcePolycubes()

    def __init__(self):
        super(EnvGrid, self).__init__()


        # self.grid = [
        #     [0, 0, 1],
        #     [0, -1, 0],
        #     [0, 0, 0]
        # ]
        # self.grid = []
        # self.grid = self.monAlgoBruteForce.CreateExplorationSpace([3, 7, 12])
        self.explorationSpace = positionsCube = np.array(
            np.meshgrid(np.arange(self.monAlgoBruteForce.sizeVoxelsCube[0]), np.arange(self.monAlgoBruteForce.sizeVoxelsCube[1]), np.arange(self.monAlgoBruteForce.sizeVoxelsCube[2]),
                        indexing='ij'))
        self.grid = np.transpose(
            [positionsCube[0].flatten(), positionsCube[1].flatten(), positionsCube[2].flatten()])

        # Starting position
        # self.y = 2
        # self.x = 0
        shape1 = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [2, 0, 1], [2, 0, 2], [3, 0, 2]])
        shape2 = np.array([[0, 0, 0], [0, 0, 1], [0, 1, 1], [0, 2, 1], [1, 2, 1]])
        shape3 = np.array([[0, 0, 0], [0, -1, 0], [0, -2, 0], [-1, -1, 0], [-2, -1, 0], [-2, -2, 0], [-2, -2, 1]])
        libraryShapes = [shape1, shape2, shape3]

        # Initialiser la table de polycubes avec un polycube choisi au hasard parmi la bibliothèque de formes et fixer son index à
        randomInit = random.randint(0, len(libraryShapes) - 1)
        initPolycube = libraryShapes[randomInit]
        indexToAdd = np.zeros(shape=len(libraryShapes[randomInit]))
        self.monAlgoBruteForce.polycubesTable = np.concatenate((initPolycube, indexToAdd[:, None]), axis=1)
        print("creation de la table ", self.monAlgoBruteForce.polycubesTable, "\n")

        # self.actions = [
        #     [-1, 0],  # Up
        #     [1, 0],  # Down
        #     [0, -1],  # Left
        #     [0, 1]  # Right
        # ]

        # create liste of actions from monAlgoBruteForce using return of PivotPolycube()
        self.actions = self.monAlgoBruteForce.PivotPolycube()
        print("actions : ", self.actions, "\n")
        # self.actions = [
        # self.monAlgoBruteForce.PivotPolycube(self.monAlgoBruteForce.polycubesTable[i], self.actions[i]) for i in range(len(self.actions))]
        # ]

    # def reset(self):
    #     """
    #         Reset world
    #     """
    #     self.y = 2
    #     self.x = 0
    #     return (self.y*3+self.x+1)

    def reset(self):
        """
            Reset world
        """
        self.monAlgoBruteForce.polycubesTable = np.array([])
        self.monAlgoBruteForce.explorationSpace = np.array([])
        self.monAlgoBruteForce.explorationSpace = self.monAlgoBruteForce.CreateExplorationSpace([0, 0, 0])
    # def step(self, action):
    #     """
    #         Action: 0, 1, 2, 3
    #     """
    #     self.y = max(0, min(self.y + self.actions[action][0],2))
    #     self.x = max(0, min(self.x + self.actions[action][1],2))
    #
    #     return (self.y*3+self.x+1) , self.grid[self.y][self.x]

    def step(self, action):
        """
            Faut que je reflichir a comment assigné une étape a chaque action
        """
        # print("action : ", action, "\n")

    def show(self):
        """
            Show the grid
        """
        print("---------------------")
        y = 0
        for line in self.grid:
            x = 0
            for pt in line:
                print("%s\t" % (pt if y != self.y or x != self.x else "X"), end="")
                x += 1
            y += 1
            print("")

    def is_finished(self):
        return self.grid[self.y][self.x] == 1

def take_action(st, Q, eps):
    # Take an action
    if random.uniform(0, 1) < eps:
        action = randint(0, 3)
    else: # Or greedy action
        action = np.argmax(Q[st])
    return action

if __name__ == '__main__':
    env = EnvGrid()
    st = env.reset()

    Q = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]
    ]

    for _ in range(100):
        # Reset the game
        st = env.reset()
        while not env.is_finished():
            #env.show()
            #at = int(input("$>"))
            at = take_action(st, Q, 0.4)

            stp1, r = env.step(at)
            #print("s", stp1)
            #print("r", r)

            # Update Q function
            atp1 = take_action(stp1, Q, 0.0)
            Q[st][at] = Q[st][at] + 0.1*(r + 0.9*Q[stp1][atp1] - Q[st][at])

            st = stp1

    for s in range(1, 10):
        print(s, Q[s])

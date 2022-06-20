import numpy as np
import random

global polycubesTable
polycubesTable = []


# Fonction pour créer un espace d'exploration correspondant à un cube
def CreateExplorationSpace(sizeVoxelsCube):
    positionsCube = np.array(
        np.meshgrid(np.arange(sizeVoxelsCube[0]), np.arange(sizeVoxelsCube[1]), np.arange(sizeVoxelsCube[2]),
                    indexing='ij'))
    return np.transpose([positionsCube[0].flatten(), positionsCube[1].flatten(), positionsCube[2].flatten()])


# Ajouter un polycube à la Polycubes table en incrémentant l'index
def AddPolycubeToPolycubesTable(polycubeToAdd):
    global polycubesTable
    maxIndex = polycubesTable[:, 3].max()
    indexToAdd = 1 + maxIndex + np.zeros(shape=len(polycubeToAdd))
    polycubesTable = np.concatenate((polycubesTable, np.concatenate((polycubeToAdd, indexToAdd[:, None]), axis=1)),
                                    axis=0)


# Fonction pour translater un polycube en x, y et z.
def TranslatePolycube(indexPolycubeToTranslate, tx=0, ty=0, tz=0):
    global polycubesTable
    polycubesTable[polycubesTable[:, 3] == indexPolycubeToTranslate] = polycubesTable[polycubesTable[:,
                                                                                      3] == indexPolycubeToTranslate] + np.array(
        [tx, ty, tz, 0])


# Fonction pour rotater un polycube en donnant son index de rotation de 0 à 23 compris
def RotatePolycube(indexPolycubeToRotate, indexRotation=0):
    global polycubesTable
    polycubeToRotate = polycubesTable[polycubesTable[:, 3] == indexPolycubeToRotate]
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
    polycubesTable[polycubesTable[:, 3] == indexPolycubeToRotate] = np.transpose([u, v, w, i])


# Fonction pour pivoter un polycube en donnant son index de rotation de 0 à 23 compris et son index de voxel pivot
# Le voxel pivot est le voxel dont la position est inchangée une fois la rotation effectuée, c'est le centre de rotation
def PivotPolycube(indexPolycubeToPivot, indexRotation=0, indexVoxelPivot=0):
    global polycubesTable
    polycubeToPivot = polycubesTable[polycubesTable[:, 3] == indexPolycubeToPivot]
    oldx, oldy, oldz = polycubeToPivot[indexVoxelPivot][:3]
    RotatePolycube(indexPolycubeToPivot, indexRotation)
    polycubeToPivot = polycubesTable[polycubesTable[:, 3] == indexPolycubeToPivot]
    newx, newy, newz = polycubeToPivot[indexVoxelPivot][:3]
    TranslatePolycube(indexPolycubeToPivot, oldx - newx, oldy - newy, oldz - newz)


# Fonction pour calculer le nombre de voxels en dehors de l'espace de travail dans la configuration actuelle
def CountVoxelsOut(explorationSpace):
    global polycubesTable
    nbVoxelsOut = 0
    for eachVoxel in polycubesTable[:, :3]:
        nbVoxelsOut = nbVoxelsOut + np.any(np.all(eachVoxel == explorationSpace[:, :3], axis=1))
    return nbVoxelsOut


# Fonction pour calculer le nombre de voxels en collision dans la configuration actuelle
def CountVoxelsInCollision():
    global polycubesTable
    nbCollisions = 0
    for eachVoxel in polycubesTable[:, :3]:
        # First test if the voxel is inside the exploration space
        if np.any(np.all(eachVoxel == explorationSpace[:, :3], axis=1)):
            nbCollisions = nbCollisions + np.sum(np.all(eachVoxel == polycubesTable[:, :3], axis=1)) - 1
    return nbCollisions / 2


# Fonction pour calculer le nombre de voxels remplis et uniques dans la configuration actuelle
def CountVoxelsFilled(explorationSpace):
    global polycubesTable
    nbVoxelsFilled = 0
    for eachVoxel in polycubesTable[:, :3]:
        # First test if the voxel is inside the exploration space
        if np.any(np.all(eachVoxel == explorationSpace[:, :3], axis=1)):
            nbVoxelsFilled = nbVoxelsFilled + 1
    nbVoxelsFilled = nbVoxelsFilled - CountVoxelsInCollision()
    return nbVoxelsFilled


# Fonction pour calculer le nombre de voxels laissés pour vide non rempli dans la configuration actuelle
def CountVoxelsNotFilled(explorationSpace):
    return len(explorationSpace) - CountVoxelsFilled(explorationSpace)


# Fonction pour calculer le pourcentage de voxels remplissant l'espace par rapport à la taille de l'espace à remplir
# dans le contexte actuel
def CountPercentageSpaceFilled(explorationSpace):
    return CountVoxelsFilled(explorationSpace) / len(explorationSpace.shape)


# Initialiser la seed
random.seed(0)

# Initialiser l'espace d'exploration
explorationSpace = CreateExplorationSpace([3, 7, 12])

# Déclarer la librairie de formes 3D à utiliser, ici trois formes sont utilisées, mais il est possible d'en utiliser
# plus ou moins
shape1 = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [2, 0, 1], [2, 0, 2], [3, 0, 2]])
shape2 = np.array([[0, 0, 0], [0, 0, 1], [0, 1, 1], [0, 2, 1], [1, 2, 1]])
shape3 = np.array([[0, 0, 0], [0, -1, 0], [0, -2, 0], [-1, -1, 0], [-2, -1, 0], [-2, -2, 0], [-2, -2, 1]])
libraryShapes = [shape1, shape2, shape3]

# Initialiser la table de polycubes avec un polycube choisi au hasard parmi la bibliothèque de formes et fixer son
# index à
randomInit = random.randint(0, len(libraryShapes) - 1)
initPolycube = libraryShapes[randomInit]
indexToAdd = np.zeros(shape=len(libraryShapes[randomInit]))
polycubesTable = np.concatenate((initPolycube, indexToAdd[:, None]), axis=1)
print("creation de la table ",polycubesTable, "\n")

AddPolycubeToPolycubesTable(libraryShapes[2])
print("ajout d'un polycube à la table, avant translation",polycubesTable, "\n")

TranslatePolycube(1, 2, 0, 2)
print("polycubesTable apres Translation du 2eme polycube avec 2,0,2", polycubesTable, "\n")

# RotatePolycube(1, 12)
# print(polycubesTable, "\n")

PivotPolycube(0, 12, 3)
print("polycubesTable apres pivot du polycube index 0 avec la rotation 12 (u, v, w = -v, u, w) selon le pivot 3",polycubesTable, "\n")

TranslatePolycube(0, 0, -3, 2)
print(polycubesTable, "\n")

TranslatePolycube(0, 0, 2, 0)
TranslatePolycube(1, 0, 2, 0)

print(polycubesTable, "\n")

print("C1 : ", CountVoxelsOut(explorationSpace))
print("C2 : ", CountVoxelsInCollision())
print("C3 : ", CountVoxelsFilled(explorationSpace))
print("C4 : ", CountVoxelsNotFilled(explorationSpace))
print("C5 : ", CountPercentageSpaceFilled(explorationSpace))



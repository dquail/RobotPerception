import numpy
import random
random.seed(0)
from tiles import *

def tileCode(numTilings, vectorLength, value):
    indexes = tiles(numTilings, vectorLength, value)
    print("Tilecode for :" + str(value) + str(indexes))
    featureVector = numpy.zeros(vectorLength)
    for idx in indexes:
        featureVector[idx] = 1
    return featureVector

"""

def tileCode(observation):
    #Very simple implementation taking the state (measured as the encoder position and returning a vector

    featureVector = numpy.zeros(103)
    idx = observation / 10
    featureVector[idx] = 1

    return featureVector

"""
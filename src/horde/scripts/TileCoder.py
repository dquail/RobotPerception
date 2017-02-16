import numpy
import random
random.seed(0)
from tiles import *

def tileCode(numTilings, vectorLength, value):
    indexes = tiles(numTilings, vectorLength, value)
    #print("Tilecode for :" + str(value) + str(indexes))
    featureVector = numpy.zeros(vectorLength)
    for idx in indexes:
        featureVector[idx] = 1
    return featureVector

class TileCoder(object):

    numberOfTiles = 8
    numberOfTilings = 8

    @staticmethod
    def getIndexes(numTilings, vectorLength, value):
        indexes = tiles(numTilings, vectorLength, value)
        return indexes, vectorLength

    @staticmethod
    def getVectorFromIndexes(indexes, vectorLength):
        featureVector = numpy.zeros(vectorLength)
        for idx in indexes:
            featureVector[idx] = 1
        return featureVector

    @staticmethod
    def getFeatureVectorFromValues(value, numTilings = numberOfTilings, numTiles = numberOfTiles):
        vectorLength = numTilings * numpy.power(numTiles, len(value))
        indexes, l = TileCoder.getIndexes(numTilings, vectorLength, value)
        featureVector = TileCoder.getVectorFromIndexes(indexes, vectorLength)
        return featureVector

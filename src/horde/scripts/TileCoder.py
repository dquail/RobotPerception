import numpy

def tileCode(observation):
    #Very simple implementation taking the state (measured as the encoder position and returning a vector

    featureVector = numpy.zeros(103)
    idx = observation / 10
    featureVector[idx] = 1

    return featureVector
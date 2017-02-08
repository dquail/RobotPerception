from TDLambda import *

class StepsToLeftDemon(TDLambda):
    def __init__(self, featureVectorLength, alpha):
        TDLambda.__init__(self, featureVectorLength, alpha)

    def gamma(self, state, observation):
        encoder = observation['encoder']

        if (encoder == -1):
            return 0
        elif (encoder == 1023):
            #This represents the extreme position
            return 0
        else:
            return 1

    def cumulant(self, encoder, speed, load = 0):
        return 1
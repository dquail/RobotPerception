from TDLambda import *

class StepsToLeftDemon(TDLambda):
    def __init__(self, featureVectorLength, alpha):
        TDLambda.__init__(self, featureVectorLength, alpha)

    def gamma(self, state, observation):
        if (observation == -1):
            return 0
        elif (observation == 1023):
            #This represents the extreme position
            print("EXTREME LEFT POSITION. GAMMA 0")
            return 0
        else:
            return 1

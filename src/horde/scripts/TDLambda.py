import numpy

class TDLambda:
    def __init__(self, featureVectorLength, alpha):
        #set up lambda, gamma, etc.
        self.numberOfFeatures = featureVectorLength
        self.lastState = 0
        self.lastObservation = 0
        self.weights = numpy.zeros(self.numberOfFeatures)
        self.eligibilityTrace = numpy.zeros(self.numberOfFeatures)
        self.gammaLast = 1

        self.alpha = alpha
        self.lastAction = 0

    def lam(self, state, observation):
        return 0.95

    def gamma(self, state, observation):
        if (observation == -1):
            return 0
        else:
            return 1

    def cummulant(self, state, observation):
        return 1

    def learn(self, lastState, newState, observation):

        zNext = self.cummulant(newState, observation)
        gammaNext = self.gamma(newState, observation)
        lam = self.lam(newState, observation)

        self.eligibilityTrace = self.gammaLast * lam * self.eligibilityTrace + lastState
        tdError = zNext + gammaNext * numpy.inner(newState, self.weights) - numpy.inner(lastState, self.weights)
        self.weights = self.weights + self.alpha * tdError * self.eligibilityTrace


        self.gammaLast = gammaNext

    def prediction(self, state):
        return numpy.inner(self.weights, state)

"""
Test code
l = TDLambda(100, 0.05)
firstState = [0]*100
firstState[2] = 1
firstState[75] = 1
secondState = [0]*100
secondState[75] = 1
secondState[5] = 1
thirdState = [0]*100
thirdState[21] = 1
thirdState[33] = 1
fourthState = [0]*100
fourthState[9] = 1
fourthState[98] = 1
l.learn(firstState, secondState, 2)
l.learn(secondState, thirdState, -1)
l.learn(thirdState, fourthState, 2)

"""

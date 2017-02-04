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

    def learn(self, lastState, action, newState, observation):
        print("!!!!! LEARN  !!!!!!!")
        print("lastState: " )
        print(lastState)
        print("action")
        print(action)
        print("NewState")
        print(newState)
        print("Observation:" + str(observation))
        print("Weights before:")
        print(self.weights)
        zNext = self.cummulant(newState, observation)
        print("Cummulant: " + str(zNext))
        gammaNext = self.gamma(newState, observation)
        print("gammaNext: " + str(gammaNext))
        lam = self.lam(newState, observation)
        print("gammaLast: " + str(self.gammaLast))

        print("lambda: " + str(lam))
        print("Eligibility before:")
        print(self.eligibilityTrace)
        self.eligibilityTrace = self.gammaLast * lam * self.eligibilityTrace + lastState
        print("Eligibility trace after:")
        print(self.eligibilityTrace)
        tdError = zNext + gammaNext * numpy.inner(newState, self.weights) - numpy.inner(lastState, self.weights)

        print("tdError: " + str(tdError))
        print("Weights before:")
        print(self.weights)
        self.weights = self.weights + self.alpha * tdError * self.eligibilityTrace

        print("wEights after: ")
        print(self.weights)
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

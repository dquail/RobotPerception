import numpy
from TDLambda import *

class GTDLambda(TDLambda):
    def __init__(self, featureVectorLength, alpha):
        TDLambda.__init__(self, featureVectorLength, alpha)
        self.hWeights = numpy.zeros(featureVectorLength)

    def rho(self):
        #TO be overwritten based on the target policy
        return 1

    def learn(self, lastState, action, newState, observation):
        print("!!!!! LEARN  !!!!!!!")
        print("alpha: " + str(self.alpha))
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
        rho = self.rho(action)
        print("rho: " + str(rho))
        self.eligibilityTrace = rho * (self.gammaLast * lam * self.eligibilityTrace + lastState)
        print("Eligibility trace after:")
        print(self.eligibilityTrace)
        tdError = zNext + gammaNext * numpy.inner(newState, self.weights) - numpy.inner(lastState, self.weights)

        print("tdError: " + str(tdError))

        print("hWeights before:")
        print(self.hWeights)

        self.hWeights = self.hWeights + self.alpha * 0.1 * (tdError * self.eligibilityTrace - (numpy.inner(self.hWeights, lastState)) * lastState)

        print("hWeights after:")
        print(self.hWeights)

        print("Weights before:")
        print(self.weights)

        self.weights = self.weights + self.alpha * (tdError * self.eligibilityTrace - gammaNext * (1-lam)  * (numpy.inner(self.eligibilityTrace, self.hWeights) * newState))

        print("wEights after: ")
        print(self.weights)
        pred = self.prediction(newState)
        print("Prediction: " + str(pred))
        self.gammaLast = gammaNext


class TDLambda:
    def __init__(self):
        #set up lambda, gamma, etc.

        self.lastState = 0
        self.lastObservation = 0

        self.lastAction = 0

    def labmda(self, state, observation):
        return 1

    def gamma(self, state, observation):
        return 1

    def cummulant(self):
        return 1


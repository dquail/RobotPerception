import rospy

from std_msgs.msg import Int16

class Verifier:
    def __init__(self, bufferLength):
        self.bufferLength = bufferLength
        self.gammas = []
        self.cumulants = []
        self.predictions = []


    def append(self, gamma, cumulant, prediction):
        print("Verifiying with gamma: " + str(gamma), ", cumulant: " + str(cumulant) + ", prediction: " + str(prediction))
        if (len(self.gammas) == self.bufferLength):
            #Remove the first item from the arrays
            self.gammas.pop(0)
            self.cumulants.pop(0)
            self.predictions.pop(0)

        self.gammas.append(gamma)
        self.cumulants.append(cumulant)
        self.predictions.append(prediction)

        #First gamma has to be 1 since the return of first is always just the unweighted cumulant

        self.gammas[0] = 1
        runningGamma = 1
        runningCumulant = 0

        for i in range(0, len(self.gammas)):
            runningCumulant = runningCumulant + self.cumulants[i] * runningGamma
            runningGamma = runningGamma * self.gammas[i]
            #TODO jump out of for loop if runningGamma = 0 since no point


        predict = self.predictions[0]
        print("Prediction: " + str(self.predictions[0]))

        print("Actual: " + str(runningCumulant))

        #Publish the values
        """
        pubPrediction = rospy.Publisher('horde_verifier/predicted', Int16, queue_size=10)
        pubPrediction.publish(self.predictions[0])

        pubActual = rospy.Publisher('horde_verifier/actual', Int16, queue_size=10)
        pubActual.publish(runningCumulant)

        pubError = rospy.Publisher('horde_verifier/error', Int16, queue_size=10)
        pubError.publish(self.predictions[0] - runningCumulant)
        """



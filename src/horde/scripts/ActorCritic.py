import numpy
import rospy
from std_msgs.msg import Float64
from horde.msg import StateRepresentation
from TileCoder import *

class ActorCritic:
    def __init__(self):
        self.actions = [1,2]
        self.numberOfFeatures = TileCoder.numberOfTilings * TileCoder.numberOfTiles * TileCoder.numberOfTiles

        #Eligibility traces and weights
        self.elibibilityTraceValue = numpy.zeros(self.numberOfFeatures)
        self.valueWeights = numpy.zeros(self.numberOfFeatures)
        self.elibibilityTracePolicy = numpy.zeros(self.numberOfFeatures * len(self.actions))
        self.policyWeights = numpy.zeros(self.numberOfFeatures * len(self.actions))

        self.lambdaPolicy = 0.35
        self.lambdaValue = 0.35
        self.averageReward = 0.0
        self.alpha = 0.01
        self.beta = 0.01
        self.rewardStep = 0.001

    def policyFeatureVectorFromStateAction(self, state, action):

        if (action == 1):
            #move left
            featureActionVector = numpy.append(state.X, numpy.zeros(len(state.X)))

        if (action == 2):
            #move right
            featureActionVector = numpy.append(numpy.zeros(len(state.X)), state.X)


        return featureActionVector

    def policy(self, state, action):
        print("---- policy(" + str(state.encoder) + ", " + str(action) + ")")

        #returns the probability of taking this action
        sumOfExponents = 0.0

        #calculate the sum of the exponents
        for a in self.actions:
            #featureVector = TileCoder.getFeatureVectorFromValues([((self.motoEncoder- self.minEncoder)/(self.maxEncoder-self.minEncoder)) * TileCoder.numberOfTiles, ((self.speed + self.maxSpeed) / (self.maxSpeed - self.minSpeed)) * TileCoder.numberOfTiles])
            actionFeatureVector = self.policyFeatureVectorFromStateAction(state, a)
            sumOfExponents = sumOfExponents + numpy.exp(numpy.inner(self.policyWeights, actionFeatureVector))
        print("sumOfExponents: " + str(sumOfExponents))
        actionFeatureVector = self.policyFeatureVectorFromStateAction(state, action)
        probability = numpy.exp(numpy.inner(self.policyWeights, actionFeatureVector)) / sumOfExponents
        print("-Returned probability: " + str(probability))
        return probability

    def pickActionForState(self, state):
        print("******* pickActionForState ************")
        policyArray = numpy.zeros(len(self.actions))

        i = 0
        for action in self.actions:
            policyArray[i] = self.policy(state, action)
            i = i + 1

        #select action given this distribution

        print("policy distribution array: " + str(policyArray))
        action = numpy.random.choice(numpy.arange(1, len(self.actions)+1), p=policyArray)
        print("Action chosen: " + str(action))
        print("******* pickActionForState **********")
        print("-")
        return action

    #This should probably not be defined with the actor critic, but rather be sent to the actor critic in the learn step
    def reward(self, previousState, action, newState):
        if ((action == 1) & (newState.encoder < 550.0) & (previousState.encoder > 550.0)):
            return 1
        else:
            return 0

    def sumOfProbTimesFeatures(self, state):
        sumVector = numpy.zeros(self.numberOfFeatures * len(self.actions))
        for action in self.actions:
            sumVector = sumVector + numpy.inner(self.policy(state, action), self.policyFeatureVectorFromStateAction(state, action))

        return sumVector

    def learn(self, previousState, action, newState):
        print("============= In actor critic learn =========")

        reward = self.reward(previousState, action, newState)
        print("previous encoder: " + str(previousState.encoder) + ", new encoder: " + str(newState.encoder) + ", action: " + str(action) + ", reward: " + str(reward))

        #Critic update
        tdError = reward - self.averageReward + numpy.inner(newState.X, self.valueWeights) - numpy.inner(previousState.X, self.valueWeights)
        print("tdError: " + str(tdError))
        self.averageReward = self.averageReward + self.rewardStep * tdError
        print("Average reward: " + str(self.averageReward))
        self.elibibilityTraceValue = self.lambdaValue * self.elibibilityTraceValue + previousState.X
        self.valueWeights = self.valueWeights + self.beta * tdError * self.elibibilityTraceValue

        #Actor update
        self.elibibilityTracePolicy = self.lambdaPolicy * self.elibibilityTracePolicy + self.policyFeatureVectorFromStateAction(previousState, action) - self.sumOfProbTimesFeatures(previousState)
        self.policyWeights = self.policyWeights + self.alpha * tdError * self.elibibilityTracePolicy

        print("============ End actor critic learn ============")
        print("-")



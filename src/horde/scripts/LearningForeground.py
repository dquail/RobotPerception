#!/usr/bin/env python

"""
Author: David Quail, February, 2017.

Description:
LearningForeground contains a collection of GVF's. It accepts new state representations, learns, and then takes action.

"""


import rospy
import threading
import yaml

from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float64

from dynamixel_driver.dynamixel_const import *

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList

from horde.msg import StateRepresentation

from BehaviorPolicy import *
from TDLambda import *
from TileCoder import *
from GVF import *
from StepsToLeftDemon import *
from DirectStepsToLeftDemon import *
from Verifier import *
from PredictLoadDemon import *

import numpy

"""
sets up the subscribers and starts to broadcast the results in a thread every 0.1 seconds
"""

def directLeftPolicy(state):
    return 2

def atLeftGamma(state):
    if state.encoder == 1023:
        return 0
    else:
        return 1

class LearningForeground:

    def __init__(self):
        self.demons = []
        self.verifiers = []

        self.behaviorPolicy = BehaviorPolicy()

        self.lastAction = 0

        """
        extremeLeftPrediction = StepsToLeftDemon(self.numTiles*self.numTiles*self.numTilings, 1.0/(10.0 * self.numTilings))
        extremeLeftVerifier = Verifier(0)
        self.verifiers.append(extremeLeftVerifier)
        self.demons.append(extremeLeftPrediction)
        """

        directLeftPrediction = GVF(TileCoder.numberOfTiles*TileCoder.numberOfTiles * TileCoder.numberOfTilings, 0.1/TileCoder.numberOfTilings, True)
        directLeftPrediction.policy = directLeftPolicy
        directLeftPrediction.gamma = atLeftGamma
        self.demons.append(directLeftPrediction)
        """
        directLeftPrediction = DirectStepsToLeftDemon(self.numTiles*self.numTiles*self.numTilings, 1.0/(10.0 * self.numTilings))
        self.demons.append(directLeftPrediction)
        #Nothing can really be learned from off policy verifiers
        #directLeftVerifier = Verifier(50)
        #self.verifiers.append(directLeftVerifier)
        """

        """
        predictLoadPrediction = PredictLoadDemon(self.numTiles*self.numTiles*self.numTilings, 1.0/(10.0 * self.numTilings))
        predictLoadVerifier = Verifier(5)
        self.demons.append(predictLoadPrediction)
        self.verifiers.append(predictLoadVerifier)
        """

        self.previousState = False

        #Initialize the sensory values of interest

    def performAction(self, action):
        #Take the action and issue the actual dynamixel command
        pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)

        if (action ==1):
            #Move left
            pub.publish(0.0)
        elif (action == 2):
            pub.publish(3.0)

        self.lastAction = action

    def updateDemons(self, newState):
        #print("LearningForeground received stateRepresentation: " + str(newState))

        encoderPosition = newState.encoder
        speed = newState.speed
        load = newState.load

        if not self.previousState:
            #We can't learn unless we have an initial state.
            self.previousState = newState
        else:
            action  = self.behaviorPolicy.policy(newState)

            self.performAction(action)

            #Learning
            for i in range(0, len(self.demons)):
                self.demons[i].learn(self.previousState, self.lastAction, newState)
                if (len(self.verifiers) > i):
                    self.verifiers[i].append(self.demons[i].gamma(newState), self.demons[i].cumulant(newState), self.demons[i].prediction(newState))


    def publishPredictions(self):
        print("Publishing predictions")


    def receiveStateUpdateCallback(self, newState):
        #Staterepresentation callback
        #Convert the list of X's into an actual numpy array
        newState.X = numpy.array(newState.X)
        newState.lastX = numpy.array(newState.lastX)
        self.updateDemons(newState)
        self.publishPredictions()
        self.previousState = newState

    def start(self):
        print("In Horde foreground start")
        rospy.init_node('horde_foreground', anonymous=True)
        # Subscribe to all of the relevent sensor information. To start, we're only interested in motor_states, produced by the dynamixels
        #rospy.Subscriber("observation_manager/servo_position", Int16, self.receiveObservationCallback)
        rospy.Subscriber("observation_manager/state_update", StateRepresentation, self.receiveStateUpdateCallback)

        rospy.spin()

if __name__ == '__main__':
    foreground = LearningForeground()
    foreground.start()


"""
motor_states:
  -
    timestamp: 1485931061.8
    id: 2
    goal: 805
    position: 805
    error: 0
    speed: 0
    load: 0.0
    voltage: 12.3
    temperature: 32
    moving: False
  -
    timestamp: 1485931061.8
    id: 3
    goal: 603
    position: 603
    error: 0
    speed: 0
    load: 0.0
    voltage: 12.3
    temperature: 34
    moving: False
"""
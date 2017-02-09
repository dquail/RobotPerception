#!/usr/bin/env python

"""
Author: David Quail, February, 2017.

Description:
Horde will contain a collection of demons as well as the behavior policy

It receives observations from the observation manager (at intervals determined by the interval).
With those observations it can make next actions as well as have each demon learn from the previous observation, action

"""


import rospy
import threading
import yaml
import json

from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float64

from dynamixel_driver.dynamixel_const import *

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList

from BehaviorPolicy import *
from TDLambda import *
from TileCoder import *
from StepsToLeftDemon import *
from DirectStepsToLeftDemon import *
from Verifier import *
from PredictLoadDemon import *
"""
sets up the subscribers and starts to broadcast the results in a thread every 0.1 seconds
"""

class Horde:

    def __init__(self):
        self.demons = []
        self.verifiers = []

        self.behaviorPolicy = BehaviorPolicy()
        self.previousAction = 0
        self.numTiles = 8
        self.numTilings = 8

        """
        extremeLeftPrediction = StepsToLeftDemon(self.numTiles*self.numTiles*self.numTilings, 1.0/(10.0 * self.numTilings))
        extremeLeftVerifier = Verifier(0)
        self.verifiers.append(extremeLeftVerifier)
        self.demons.append(extremeLeftPrediction)
        """

        directLeftPrediction = DirectStepsToLeftDemon(self.numTiles*self.numTiles*self.numTilings, 1.0/(10.0 * self.numTilings))
        self.demons.append(directLeftPrediction)
        #Nothing can really be learned from off policy verifiers
        #directLeftVerifier = Verifier(50)
        #self.verifiers.append(directLeftVerifier)

        """
        predictLoadPrediction = PredictLoadDemon(self.numTiles*self.numTiles*self.numTilings, 1.0/(10.0 * self.numTilings))
        predictLoadVerifier = Verifier(5)
        self.demons.append(predictLoadPrediction)
        self.verifiers.append(predictLoadVerifier)
        """

        self.previousState = []

        #Initialize the sensory values of interest

    def performAction(self, action):
        #Take the action and issue the actual dynamixel command
        pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)

        if (action ==1):
            #Move left
            pub.publish(0.0)
        elif (action == 2):
            pub.publish(3.0)


    # Sensor callbacks
    def receiveObservationCallback(self, data):
        #TODO - Terrible conversion from Int16 to int. Need to fix this.
        print("Horde received data: " + str(data))
        jsonObservation= json.loads(data.data)
        encoderPosition = jsonObservation['encoder']
        speed = jsonObservation['speed']
        load = jsonObservation['load']

        if len(self.previousState) == 0:
            #We can't learn unless we have an initial state.
            self.previousState = tileCode(self.numTilings, self.numTilings * self.numTiles * self.numTiles, [((encoderPosition-510.0)/(1023.0-510.0)) * self.numTiles, ((speed + 200.0) / 400.0) * self.numTiles])
            self.previousAction = 0
        else:
            observation = encoderPosition
            action  = self.behaviorPolicy.policy()

            self.performAction(action)

            #Learning
            featureVector = tileCode(self.numTilings, self.numTilings * self.numTiles * self.numTiles, [((encoderPosition-510.0)/(1023.0-510.0)) * self.numTiles, ((speed + 200.0) / 400.0) * self.numTiles])
            #featureVector = tiles(self.numTilings, self.numTilings * self.numTiles, [(observation/1023) * self.numTiles])
            for i in range(0, len(self.demons)):
                self.demons[i].learn(self.previousState, self.previousAction, featureVector, jsonObservation)
                if (len(self.verifiers) > i):
                    self.verifiers[i].append(self.demons[i].gamma(featureVector, jsonObservation), self.demons[i].cumulant(featureVector, jsonObservation), self.demons[i].prediction(featureVector), jsonObservation)

            self.previousState = featureVector
            self.previousAction = action
        """
        movingLeftFeatureVector = tileCode(self.numTilings, self.numTilings * self.numTiles * self.numTiles,
                             [((700 - 510.0) / (1023.0 - 510.0)) * self.numTiles,
                              ((-150 + 200.0) / 400.0) * self.numTiles])
        movingRightFeatureVector = tileCode(self.numTilings, self.numTilings * self.numTiles * self.numTiles,
                             [((700 - 510.0) / (1023.0 - 510.0)) * self.numTiles,
                              ((150 + 200.0) / 400.0) * self.numTiles])
        leftPrediction = self.demons[0].prediction(movingLeftFeatureVector)
        rightPrediction = self.demons[0].prediction(movingRightFeatureVector)
        print"################"
        print("Moving right prediction: " + str(rightPrediction))
        print("Moving left prediction: " + str(leftPrediction))
        print("###############")
        """

    def start(self):
        print("In Horde start")
        rospy.init_node('horde', anonymous=True)
        # Subscribe to all of the relevent sensor information. To start, we're only interested in motor_states, produced by the dynamixels
        #rospy.Subscriber("observation_manager/servo_position", Int16, self.receiveObservationCallback)
        rospy.Subscriber("observation_manager/observation", String, self.receiveObservationCallback)

        rospy.spin()

if __name__ == '__main__':
    horde = Horde()
    horde.start()


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
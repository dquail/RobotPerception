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
from Verifier import *

"""
sets up the subscribers and starts to broadcast the results in a thread every 0.1 seconds
"""

class Horde:

    def __init__(self):
        self.demons = []
        self.verifiers = []

        self.behaviorPolicy = BehaviorPolicy()

        extremeLeftPrediction = StepsToLeftDemon(103, 0.05)
        extremeLeftVerifier = Verifier(50)

        self.demons.append(extremeLeftPrediction)
        self.verifiers.append(extremeLeftVerifier)

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
        val = data.data

        if len(self.previousState) == 0:
            #We can't learn unless we have an initial state.
            self.previousState = tileCode(val)

        else:
            observation = val
            action  = self.behaviorPolicy.policy()

            self.previousAction = action
            self.performAction(self.previousAction)

            #Learning
            featureVector = tileCode(observation)

            for i in range(0, len(self.demons)):

                self.demons[i].learn(self.previousState, self.previousAction, featureVector, observation)
                self.verifiers[i].append(self.demons[i].gamma(featureVector, observation), self.demons[i].cummulant(featureVector, observation), self.demons[i].prediction(featureVector), observation)

            self.previousState = featureVector



    def start(self):
        print("In Horde start")
        rospy.init_node('horde', anonymous=True)
        # Subscribe to all of the relevent sensor information. To start, we're only interested in motor_states, produced by the dynamixels
        rospy.Subscriber("observation_manager/servo_position", Int16, self.receiveObservationCallback)


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
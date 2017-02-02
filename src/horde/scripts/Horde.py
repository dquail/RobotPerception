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

from dynamixel_driver.dynamixel_const import *

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList

"""
sets up the subscribers and starts to broadcast the results in a thread every 0.1 seconds
"""

class Horde:

    def __init__(self):
        self.demons = []
        #Initialize the sensory values of interest


    # Sensor callbacks
    def receiveObservationCallback(self, data):
        print("Horde received data" + str(data))

        print("do some action")
        print("Do the learning for each demon")

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
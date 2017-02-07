#!/usr/bin/env python

"""
Author: David Quail, February, 2017.

Description:
The observation manager will
1. listen for various different sensor data coming from various different data sources (servos, cameras, etc).
2. Maintain the data structure of most recent observations across data sources sources.
3. Publish the most recent observations at a predictable time interval.

By massaging the sensory data and publishing it at frequent, predictable time intervals, a learning system can subscribe
to this message. Once subscribed, the learner can perform control and learning at this interval, rather than having
to poll for the information after actions.

TODO:
Publish a message of specific format that encapsulates all the data of interest. Not just a primitive int of angle
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

class ObservationManager:

    def __init__(self):
        self.publishingFrequency = 0.2 #Seconds between updates
        #Initialize the sensory values of interest
        self.motoEncoder = 0
        self.speed = 0

    # Sensor callbacks
    def motorStatesCallback(self, data):
        print("In observation_manager callback")

        self.motoEncoder = data.motor_states[0].position
        self.speed = data.motor_states[0].speed

    def publishObservation(self):
        print("In publishObservation with value of publishing: " + str(self.motoEncoder))
        # Test republishing the message for plotting purposes
        #TODO - publish the speed as well

        pubObservation = rospy.Publisher('observation_manager/observation', String, queue_size = 10)
        msgJSON = '{"encoder":' + str(self.motoEncoder) + ', "speed":' + str(self.speed) + '}'
        msg = String()
        msg.data = msgJSON
        pubObservation.publish(msg)

        pub = rospy.Publisher('observation_manager/servo_position', Int16, queue_size=10)
        pub.publish(self.motoEncoder)
        #pub = rospy.Publisher('observation_manager_observation/moto_encoder', Int16, queue_size=10)
        #pub.publish(self.motoEncoder)
        threading.Timer(self.publishingFrequency, self.publishObservation).start()

    def start(self):
        print("In listener")
        rospy.init_node('observation_manager', anonymous=True)
        # Subscribe to all of the relevent sensor information. To start, we're only interested in motor_states, produced by the dynamixels
        rospy.Subscriber("motor_states/pan_tilt_port", MotorStateList, self.motorStatesCallback)

        self.publishObservation()

        #rospy.spin()


if __name__ == '__main__':
    manager = ObservationManager()
    manager.start()


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
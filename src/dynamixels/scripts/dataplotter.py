#!/usr/bin/env python
import rospy
import yaml

from std_msgs.msg import String
from std_msgs.msg import Int16

from dynamixel_driver.dynamixel_const import *

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList

def callback(data):
    print("In callback")
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    #print(data)

    print(data.motor_states[0].id)

    #Test republishing the message for plotting purposes
    pub = rospy.Publisher('dyna_logging', Int16, queue_size=10)
    pub.publish(data.motor_states[0].temperature)
    #jsond = yaml.load(data)
    #print(jsond)


def listener():
    print("In listener")
    rospy.init_node('data_listener', anonymous = True)
    rospy.Subscriber("motor_states/pan_tilt_port", MotorStateList, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()

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
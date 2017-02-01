#Using ROS to monitor and control Dynamixel servos
There's a reasonable amount of documentation on the ROS wiki http://wiki.ros.org/dynamixel_controllers/ that allows a person to get up and running with ROS + Dynamixel's. But it assumes a working knowledge of ROS ... something I didn't have, given this was my first project with the framework. So I'm putting this guide together, hoping it might be useful for someone looking to control their Dynamixel servos with ROS, and who has little to no experience with ROS. Getting to a point where I could control the servos and monitor and plot their behavior took about 4 hours of my time. Hopefully this will save you some time.

##Prerequesites
The only assumption that I have is that you have ROS (I used Indigo) installed, as well as Python (I used 2.7). Almost all of the issues I encountered were related to mismatched versions of Python packages. I spent a lot of time forcing ROS to use 2.7 as well as the proper versions of packages like pyserial. Someone more familiar with Anaconda, pip, and linux fundamentals will have a much easier time spelunking through these issues. 

##Getting started

##Common commands
*Currently required for each terminal tab
$source activate py27
$source ./devel/setup.bash

###1. Create the workspace folder structure.
A workspace is where common packages for your project will exist. You can have multiple workspaces spread across your filesystem.   
#For me, I'm creating a project that will eventually allow a robotic digger to learn the optimal way to dig sand. Hence the name of the workspace
```python
$mkdir -p RobotDigger
$cd RobotDigger
$mkdir src
```

###2. Initialize the workspace:
```python
$cd src
$catkin_init_workspace
```

###3. Build the workspace:
*This will create the required files needed for ROS to run your packages
```
$cd ..
$catkin_make
```

*Note - I needed to pip install catkin_pkg

###4. Create a package:
We want to test creating a simple package that publishes and subscribes to messages. This isn't directly used by our project but it gives us a basic understanding of publish/subscribe.
 
```
$catkin_create_pkg robot_digger std_msgs rospy roscpp
```

###5. Add the workspace to your ROS environment
```
$source devel/setup.bash
```

*NOTE I need to do this in each terminal window! There's obviously a way to do so without having to type this into a new terminal window each time ... but I don't know what it is at the moment.

###6. Edit the resulting src/robot_digger/package.xml
This gets you into the habit of including a proper description, for your package.

###7. Create a publisher node
```python
#Create a script folder at same level as src
$mkdir scripts 
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
#Remember to always make these executable
$ chmod +x talker.py
```
###8. Create the subscriber node
```python
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
$ chmod +x listener.py
```

###9. Create the package
```python
$cd ..
$catkin_make
```

###10. Start the publisher
```python
#Make sure roscore is running
$roscore
$rosrun robot_digger talker.py
```
*I needed to pip install rospkg

###10. Install dynamixel_ros - Creating a different test package
```python
$sudo apt-get install ros-indigo-dynamixel-motor
```

###11. Create test package for dynamixel
```python
$catkin_create_pkg my_dynamixel_tutorial dynamixel_controllers std_msgs rospy roscpp
```
###12. Create the launch file for easily launching the controller
```python
$mkdir launch
$cd launch
```python

###13. Create controller_manager.launch with:
```xml
<!-- -*- mode: XML -*- -->

<launch>
    <node name="dynamixel_manager" pkg="dynamixel_controllers" type="controller_manager.py" required="true" output="screen">
        <rosparam>
            namespace: dxl_manager
            serial_ports:
                pan_tilt_port:
                    port_name: "/dev/ttyUSB0"
                    baud_rate: 1000000
                    min_motor_id: 1
                    max_motor_id: 25
                    update_rate: 20
        </rosparam>
    </node>
</launch>
```xml

*Make sure your dev/tty port matches what's on your computer


*Use Python 2.7 (dynamixel was assuming that but by default pyhon was 3.5)
conda create -n py27 python=2.7 anaconda
source activate py27

* A whole bunch of issues around getting the right version of python. It needs to be 2.7. That's what indigo uses. So you also need to make sure that that version
is using pyserial 2.7 (an old version.)

###13. Make sure that roscore is running
```python
$roscore
```

###14. Start the controller_manager
```python
$roslaunch my_dynamixel_tutorial controller_manager.launch
```

###15. Look at the output
```python
$rostopic echo /motor_states/pan_tilt_port

###16. Control the motor
Create the yaml file tilt.yaml
```yaml
tilt_controller:
    controller:
        package: dynamixel_controllers
        module: joint_position_controller
        type: JointPositionController
    joint_name: tilt_joint
    joint_speed: 1.17
    motor:
        id: 4
        init: 512
        min: 0
        max: 1023
```

Create the launch file
```yaml
<launch>
    <!-- Start tilt joint controller -->
    <rosparam file="$(find my_dynamixel_tutorial)/tilt.yaml" command="load"/>
    <node name="tilt_controller_spawner" pkg="dynamixel_controllers" type="controller_spawner.py"
          args="--manager=dxl_manager
                --port pan_tilt_port
                tilt_controller"
          output="screen"/>
</launch>
```

Launch the controller
```python
$roslaunch my_dynamixel_tutorial start_tilt_controller.launch
```

Move the motor
```python
rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- 1.5
```

###17. Plotting the data
ROS has a nice plotting tool called rqt_plot
You can launch it using 

```python
rqt_plot
```

Once it's launched you can dynamically plot data from topics (primitive data types).
The tilt information is in yaml format so we can republish the data and have it displayed using a publish / subscriber such as the following:

```python
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
```

You can see how you could extend this to control and monitor the servos, add other actuators, introduce learning algorithms, etc, in a nice, clean decoupled way. This took about 4 hours for me to get up and going, and anyone with a stronger knowledge of UNIX and or python would have a much easier time.

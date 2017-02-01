*Currently required for each terminal tab
$source activate py27
$source ./devel/setup.bash

1. Create workspace folder structure:
$mkdir -p RobotDigger
$cd RobotDigger
$mkdir src

2. Initialize workspace:
$cd src
$catkin_init_workspace

3. Build workspace:
$cd ..
$catkin_make

#Note - I needed to pip install catkin_pkg

4. Create the package:
$catkin_create_pkg robot_digger std_msgs rospy roscpp

5. Add the workspace to your ROS environment
$source devel/setup.bash
#NOTE I need to do this in each terminal window!

6. Edit the resulting src/robot_digger/package.xml
- Including a proper description, etc.

7. Create a publisher node
$mkdir scripts #Create a script folder at same level as src
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
$ chmod +x talker.py

8. Create the subscriber node
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
$ chmod +x listener.py


9. Create the package
$cd ..
$catkin_make

10. Start the publisher
$rosrun robot_digger talker.py

#I needed to pip install rospkg
#I also need to source the workspace
$source ./devel/setup.bash

10. Install dynamixel_ros - Creating a different test package
$sudo apt-get install ros-indigo-dynamixel-motor

11. Create test package for dynamixel
$catkin_create_pkg my_dynamixel_tutorial dynamixel_controllers std_msgs rospy roscpp

12. Create the launch file for easily launching the controller
$mkdir launch
$cd launch

Create controller_manager.launch with:
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

#Make sure your dev/tty port matches what's on your computer


*Use Python 2.7 (dynamixel was assuming that but by default pyhon was 3.5)
conda create -n py27 python=2.7 anaconda
source activate py27

* A whole bunch of issues around getting the right version of python. It needs to be 2.7. That's what indigo uses. So you also need to make sure that that version
is using pyserial 2.7 (an old version.)

Spent a whole bunch of time settign up these proper versions - I suck at version management (anoconda, pip, etc) are not my old friends

13. Start roscore
$roscore

14. Start the controller_manager
$roslaunch my_dynamixel_tutorial controller_manager.launch

15. Look at the output
$rostopic echo /motor_states/pan_tilt_port
$rqt_plot

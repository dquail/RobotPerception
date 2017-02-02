* Annoying pre-reqs for any term window
$source activate py27
$source ./devel/setup.bash

*Other commands
rqt_graph
rqt_plot
1. start roscore
$roscore

2. Start dynamixel publisher
$roslaunch dynamixels controller_manager.launch

3. Make sure that the states are being published
$rostopic echo /motor_states/pan_tilt_port

4. Start the controller
$roslaunch dynamixels start_tilt_controller.launch

5. Make sure that you can move the dynamixel
rostopic pub /tilt_controller/command std_msgs/Float64 -- 1.5

6. Start the observation manager
rosrun horde ObservationManager.py

7. Make sure that you can see it publishing data:
$rostopic echo /observation_manager/servo_positions


8. Start horde
$rosrun horde Horde.py


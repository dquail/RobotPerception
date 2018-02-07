# Robot Perception
Within this repository we are attempting to achieve computational perception via Reinforcement learning and robots.

There are several parts to this and thus, several layers of documentation.

### [ROS Setup](Documentation/ROS Setup/README.md)
- Provides steps required to install ROS, and begin to send command to a dynamixel servo, as well as reading data from the servo.

### [General value function experiments](Documentation/RobotModule2/README.md)
- Describes answering 3 predictive questions about the future using 3 different general value functions.

### [Horde and Pavlovian Control](Documentation/RobotModule3/README.md)
- Presents an architecture that is capable of learning thousands of general value functions in parallel, based of just the sensorimotor information from the servos. 
- Demonstrates how to alter the behavior of the robot based on these predictions
- Demonstrates how to use error measures that reflect learning progress.

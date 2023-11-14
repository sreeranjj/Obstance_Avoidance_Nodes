# Obstance_Avoidance_Nodes
ROS2 nodes for obstacle avoidance while moving 
The objective of this project is to design a set of controllers to make a robot drive through a set of
way points, in the presence of unknown obstacles. The blue box is in a known stationary position within the
environment, while the purple object will be added by an instructor during the demo. The robot will
use onboard odometry and dead reckoning to determine its global position during the navigation.
It will be assumed that the robot starts at global position (0m, 0m) with orientation aligned with
the x-axis.

<img width="413" alt="image" src="https://github.com/sreeranjj/Obstance_Avoidance_Nodes/assets/24253653/28e01c06-007a-4446-b058-8c361e4eb9b1">


Two nodes are required for this project: 
getObjectRange: This node should detect the ranges and orientation of obstacles. It should
subscribe to the scan node and publish the vector pointing from the robot to the nearest point on
the object.

goToGoal: This node should subscribe to the odom node which determines the robots global
position from onboad sensors for you (using dead reckoning). It should also subscribe to the getO-
bjectRange node to determine if there are any obstacles that need to be avoided.

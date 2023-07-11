# ThreeLeggedBalance
Teaching a Robot New tricks - Three legged Balance mouvements. Supervised by Guillaume Bellegarda and Auke Ijkspeert at BIOROB lab at EPFL.
![Uploading 3leggif.gif…]()

# Installation
To setup rightly the workspace, please follow the **installation** part in clemson-dira [repository](https://github.com/clemson-dira/legged_control/tree/unitree_go1).   

Reminder that at any change in the legged_control package, re-run these commands:
```
catkin build legged_controllers legged_unitree_description
catkin build legged_gazebo
```

# Quick Start

1) Set your robot type as an environment variable: ROBOT_TYPE **in every new terminal**.
```
export ROBOT_TYPE=a1
```

2) In a new terminal, run the simulation:
```
roslaunch legged_unitree_description empty_world.launch
```

3) In a new terminal, load the controller:
```
roslaunch legged_controllers load_controller.launch cheater:=true
```

4) In a new terminal, start the legged_controller or legged_cheater_controller, **NOTE that you are not allowed to start the legged_cheater_controller in real hardware!**
```
rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']                   
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0"
```
And to make the robot stand
```
rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.2
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
5) In a new terminal, launch the Rviz simulation
```
roslaunch legged_unitree_description check_joint.launch
```

6) To achieve 3-legged balance, write ``pawup` inthe controller's terminal (from step 3). Type `list` to see the different possible gaits.

7) In the python files folder, run `square_pos.py` and the end effector poses for the swinging leg will be published.
The end effector topic can be visualized in Rviz by adding it to the window, as a StampedPoint.  

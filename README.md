# franka_panda_RL
## Description
ROS2 based interface to connect functional packages (yolo, libfranka, moveit) 
with tasks (layout-repeat, Sim2Real, hand guidance...)

## Branch(Alpha)
The branch will make the documents and codes more standard from four aspects:
+ Add comments in the codes as detailed as possible;
+ Check functions and ensure them as complete as possible;
+ Separate the long file to modules to improve reusability and decrease redundance;
+ Add TODO list for further check;

## Packages/Modules
### franka_action_interface
It includes description files of customized ROS action, which will be used in other packages.

### franka_control
A ROS2 package used to control the franka robot mainly based on **Moveit** and C++.

### franka_gym
A ROS2 package for reinforcement learning and is coded based on Python.

### franka_perception and franka_vision
Actually, they are the same and one is based on C++ while another is wirtten by Python. Maybe they will be merged into one in the future.

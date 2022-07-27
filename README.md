# mavbase2
Repository for basic mav functions implemented in ROS 2

## Dependencies
- ```sudo apt install ros-$ROS_DISTRO-tf-transformations```
- ```pip3 install transforms3d```

## Launch options
This repository also contains launch files used to connect MAVROS in different situations. Here are some examples on how to use them:
### Simulation in the loop (SITL)
For a simple simulation, go to your PX4-Autopiot root and type the following:

```make px4_sitl gazebo```

This should launch gazebo with a drone spawned. After that, open another terminal and run:

```ros2 launch mavbase2 px4_sitl.launch.py```

Mavros will start and connect to the simulated drone. From here just run whichever ros2 node you wish to test!
### Real life examples
For telemetry connection with the FCU, run the following:

```ros2 launch mavbase2 px4_telemetry.launch.py```

For usb connection with the FCU, run the following:

```ros2 launch mavbase2 px4_usb.launch.py```

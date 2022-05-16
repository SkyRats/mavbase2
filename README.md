# mavbase2
Repository for basic mav functions implemented in ROS 2

## Launch options
This repository also contains launch files used to connect MAVROS in different situations. Here are some examples on how to use them:
### Simulation in the loop (SITL)
For simulations, go to your PX4-Autopiot root and type the following:

```make px4_sitl gazebo```

This should launch gazebo with a drone spawned. After that, open another terminal and run:

```ros2 launch mavbase px4_sitl.launch```

Mavros will start and connect to the simulated drone. From here just run whichever ros2 node you wish to test!
### Real life examples
For telemetry connection with the FCU, run the following:

```ros2 launch mavbase px4_telemetry.launch.py```

For usb connection with the FCU, run the following:

```ros2 launch mavbase px4_usb.launch.py```

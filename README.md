# mavbase2
Repository for basic mav functions implemented in ROS 2

## Dependencies
- ROS2 running with Mavros (Recommended installation: [skyrats-workplace](https://github.com/SkyRats/skyrats-workplace/tree/ros2))
- ```sudo apt install python3-opencv```
- ```sudo apt install ros-$ROS_DISTRO-tf-transformations```
- ```pip3 install transforms3d```
- ```pip3 install pygeodesy```
- ```pip3 install scipy```

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
## Importing MAV2.py from another package
If you followed the default installation through [skyrats-workplace](https://github.com/SkyRats/skyrats-workplace/tree/ros2), just add the following lines to your python code:
``` python
sys.path.insert(0,'/home/' + os.environ["USER"] + '/skyrats_ws2/src/mavbase2')
from MAV2 import MAV2
```
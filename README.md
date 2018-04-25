# Delivery robot
Welcome to cellphone robot project! This is part of [Project Cellphone robot](https://github.com/wang3303/ros_cellphonerobot)
: a RPi-based navigation robot. In our project we decide to use Raspberry Pi 3 Model B
, a single-board computer with considerate computation power. Read [wiki](https://github.com/wang3303/delivery_bot/wiki)
for concrete details.

Out robot is able to accomplish the following task:
* Build an map while moving around
* Receive goal location from android app and navigate to the goal based on the map built

## Structure of Package
![img]()
## Interaction with Android
![img]()

## Step 0: Understand [some basics](https://github.com/wang3303/delivery_bot/wiki/Basics) and compile the package
```commandline
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release -j1
```
If you plan to run your packages on multi-machines, read [start remote node/rviz](https://github.com/wang3303/delivery_bot/wiki/remote-machine)
and [start launch file for remote machine](https://github.com/wang3303/delivery_bot/wiki/remote-launch)


## Step 1: Configure laser scanner
You ought to configure your favorite laser scanner and publish the reading in the form of [LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)
```buildoutcfg
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.
```

For our project, we use Neato XV-11 sensor. For more details, see [here](https://github.com/wang3303/delivery_bot/wiki/Laser-scanner-setup).

If you publish your message to `/scan`, you should be able to see the lidar reading visualization after running rviz.

## Step 2: Configure your motors, encoders, and PID controller
We provide two options for converting reference velocity published to topic `\cmd_vel` to actual motions of robots: 
* [a high-level ROS-package](https://github.com/wang3303/delivery_bot/wiki/Diferential-drive): This package encoder reading and output control effort through DCmotor class defined in `hardware.py`.
    
    *Note: The system will be susceptible to delay and timeout. I would recommend use external hardware with real ISR to get
    encoder readings and output PWM signal. However, it is easier to debug in ROS*

* [ROS arduino bridge](https://github.com/wang3303/delivery_bot/wiki/ROS-arduino): We use arduino UNO to run PID control for motors and let it communicate with ROS using serial communication.
If you decide not to use

*Note: If you decide to use your own package, you should also output [Odometry Message](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)* and tf from odom to
base_link.
 
## Step 3: Test your hardware

We provide two scripts to generate reference velocity for motors:

* key_publisher.py: It publishes pressed keys to topic `\action`.

* key_to_twist_ramp: It subscribes to `\action` and publishes reference speed to `\cmd_vel`.

## Step 4: SLAM!
Open `ros_cellphonerobot/launch/slam.launch` and configure your hardware nodes. 

Then run `roslaunch ros_cellphonerobot slam.launch` in your terminal.

Tweak [parameter setting](https://github.com/wang3303/delivery_bot/wiki/SLAM) in your `mapping_default.launch`. 

Run Rviz to make sure SLAM is generating desired occupancy grid in `\maphector`. After that, start the `key_publisher`
to move your robot around.

Run the following to save the occupancy grid.
```commandline
rosrun map_server map_saver -f mapname /map:=/maphector
```

## Step 5: Navigation

Change the map to be published in cellphonerobot/launch/move_base.launch

```html
  <node name="map_server" pkg="map_server" type="map_server" args="$(find ros_cellphonerobot)/map/map_name.yaml" output="screen"/>
```

Try running `roslaunch ros_cellphonerobot slam.launch` in your terminal.

Tweak your [parameter setting](https://github.com/wang3303/delivery_bot/wiki/Navigation).

* Start rviz

* Click 2D pose estimate and see if the lidar reading align with the map.

* Click 2D Nav Goal to check the planning and actual paths

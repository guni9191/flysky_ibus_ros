# flysky_ibus_ros

## Introduction

- flysky ibus decoder for ros

## Features

- Need flysky RC, receiver and Usb to ttl converter
- can port to c++ and ros2 applications easily since decoder is included in "include" directory
- reads ibus message from ttyUSBx port and converts channel data to Twist msg

## Tested Environment

- ROS melodic
- ubuntu 18.04
- flysky ibus fs-ia6b receiver + usb to ttl converter(pl2303)
- flysky i6s

## How to Run

```bash
cd ${catkin_ws}/src
git clone https://github.com/guni9191/flysky_ibus_ros.git
cd flysky_ibus_ros/launch
roslaunch flysky_ibus.launch
```

## Published Topic

- cmd_vel (geometry_msgs::Twist)
    - from -1 to 1
    - flysk i6s right joystick
        - go straight = up, go backwards = down
        - left spin = left, right spin = right

## Launch Parameters

- frame_id (string)
    - publish twist_msg frame_id
- usb_port (string)
    - usb port name
- pub_topicname_cmdvel (string)
    - publish twist_msg topicname
- ang_reverse (bool)
    - angular velocity direction reverse

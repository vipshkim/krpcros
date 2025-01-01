# krpc-ros
It transfor KSP data into ros network using krpc mod, change coordnate left hand to right hand


As you know, KSP(Kerbal Space Program) is useful simulation game.
One problem is they use left-hand coordinate(KCKF: Kerbin-Centered, Kerbin-Fixed).
This ros package turn KCKF to ECEF(Earth-Centered, Earth-Fixed) normal right-hand coordinate,
include linear and angular position and rate.

The other issue is, as i know KRPC dose not provied any acceleration data just like imu sensor.
This calcuate not only calculate it by delta-velocity devide by delta-time, but also calcuate gravitational, centrefigal and coriolis acceleration effect.
This calculation use there own body data(such as krpc.space_center.active_vessel.orbit.body.rotation_speed), that may can calculate effectively on chages there body.


It is tested on 

* normal x86 window PC
* [KSP 1.12.5](https://store.steampowered.com/app/220200/Kerbal_Space_Program/)
* [KRPC 0.5.4 ksp mod](https://github.com/krpc/krpc)
* [ubuntu 24.04 for raspberrypi](https://ubuntu.com/download/raspberry-pi)
* [raspberrypi 5](https://www.raspberrypi.com/products/raspberry-pi-5/)



## Why upload this?
Basically, for the purpose of a backup.

## What is the ROS?
See this https://space.ros.org

## How to use?
Put this package in your ros2_ws/src

and build, run just same as usual ros package projects
```
cd ~/ros2_ws
colcon build
. install/setup.bash
ros2 run krpcros talker
```

and, or before run a node, you need launch any vessel once.
node will wait until create vessel object.

after node run, it will publish data into ros2 network.

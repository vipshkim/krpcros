# krpc-ros
It transfor KSP data into ros network using krpc mod, change coordnate left hand to right hand


As you know, KSP(Kerbal Space Program) is useful simulation game.
One problem is they use left-hand coordinate(KCKF: Kerbin-Centered, Kerbin-Fixed).
This ros package turn KCKF to ECEF(Earth-Centered, Earth-Fixed) normal right-hand coordinate,
include linear and angular position and rate.

The other issue is, as i know KRPC dose not provied any acceleration data just like imu sensor.
This calculate not only calculate it by delta-velocity devide by delta-time, but also calculate gravitational, centrefigal and coriolis acceleration effect.
This calculation use there own body data(such as krpc.space_center.active_vessel.orbit.body.rotation_speed), that may can calculate effectively on chages there body.


It is tested on 

* normal x86 window PC
* [KSP 1.12.5](https://store.steampowered.com/app/220200/Kerbal_Space_Program/)
* [KRPC 0.5.4 ksp mod](https://github.com/krpc/krpc)
* [ubuntu 24.04 for raspberrypi](https://ubuntu.com/download/raspberry-pi)
* [raspberrypi 5](https://www.raspberrypi.com/products/raspberry-pi-5/)
* [ros2 jazzy](https://docs.ros.org/en/jazzy/index.html)



## Why upload this?
Basically, for the purpose of a backup.

## What is the ROS?
See this https://space.ros.org

## Install requirements
1. [ros](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
2. [numba](https://numba.readthedocs.io/en/stable/user/installing.html#installing-using-pip-on-x86-x86-64-platforms)
3. [numpy](https://numpy.org/install/)
4. [KRPC Client](https://krpc.github.io/krpc/python/client.html#installing-the-library)

## How to use?
1. Put this package in your ros2_ws/src
2. Change address on the package(i will improve this soon)
```
vim ~/ros2_ws/src/krpcros/krpcros/publisher_member_function.py
:40
```
 default value was
```
 self.conn = krpc.connect(name='krpcros', address='192.168.0.3')
```
3. Build and run just same as usual ros package projects
```
cd ~/ros2_ws
colcon build --packages-select krpcros
. install/setup.bash
ros2 run krpcros conn
```

and, or before run a node, you need launch any vessel once.
node will wait until create vessel object.

after node run, it will publish data into ros2 network.

# New Feature
Now this subscribe "krpcros/joy" for controlling custom action groups(button) 0-9 and axis(axis) 1-4
-WIP

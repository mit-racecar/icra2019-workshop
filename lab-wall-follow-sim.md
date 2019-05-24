---
title: Wall Follow Lab (Simulation)
layout: post
---

## Introduction

In this lab, you will be implementing a wall follower on a simulated version of
the racecar. Your goal is to make an autonomous controller that drives the
racecar forwards while maintaining a constant distance from a wall on either
its left or right (chosen on the fly). It should also be robust to uneven
surfaces and small errors in the LIDAR data, and it should be able to recover
from small deviations from the desired state; being too far, too close, or
too angled.

This lab is to be done individually. In the next lab, you will join your team
to get your wall follower working on the real racecar. You will be reusing the
code you write for this lab so make sure it is clean and presentable to your
teammates!

We have made a series of tests to evaluate the performance of your wall
follower. In order to test your code properly you must start out with the
template starter code. The template is still very sparse and you have plenty of
freedom to implement any algorithm you'd like so long as the inputs and outputs
are the same.

## Download the wall follower lab

Clone this repository into your catkin workspace:

    cd ~/racecar_ws/src
    git clone https://github.com/mit-rss/wall_follower_sim.git

Your directory structure should now look like

- ```[YOUR_WORKSPACE]/src/```
  - ```CMakeLists.txt```
  - ```racecar_simulator/```
  - ```wall_follower_sim/```

Then rebuild your workspace with ``catkin_make```:

    cd ~/racecar_ws
    catkin_make
    source devel/setup.bash

## Simulator

First take a look at the simulator. You can launch it by running:

    roslaunch racecar_simulator simulate.launch
    
To see the simulated car, you will need to open
[```rviz```](http://wiki.ros.org/rviz) by typing

    rviz

_If you are using the Docker image, rviz should already be configured properly!_

In the left panel on the bottom click the "Add" button, and then in the
"By display type" tab click "RobotModel". You should see a small blue car
appear. Then click "Add" again and in the "By topic" tab click add the "/map"
topic and then repeat to add the laser scan topic. Under added LaserScan
dropdown menu there should be a field called "Size (m)". Change this to 0.1 so
you can see the laser scan more clearly. The checkboxes turn on and off display
types which may be useful as you add topics to visualize.

![Add button](https://i.imgur.com/85tY4tZ.png)

You should see a car in a map (walls are black, empty space is grey) and points
on that map representing the points hit by the car's lidar.

![The racecar in the starting position](https://raw.githubusercontent.com/mit-racecar/racecar_simulator/master/media/racecar_simulator_rviz_1.png)

You can move the car around by plugging in a joystick into your computer, or by
clicking the "2D Pose Estimate" button on top of rviz and dragging somewhere on
the map.

Note that the simulator does not include collision detection.

## Autonomous driving

In order to make the car drive autonomously you will need to publish messages of
type [```AckermannDriveStamped```](http://docs.ros.org/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) to the ```/drive``` topic.
    
Import the ```AckermannDriveStamped``` type like this in your wall_follower.py file:

    from ackermann_msgs.msg import AckermannDriveStamped
    
## LIDAR

The racecar (and it’s simulation) are equipped a with LIDAR sensor that measures
the distance from the racecar to its surroundings with high accuracy. All of the
LIDAR data is published to the ```/scan``` topic.

The data is of type [```LaserScan```](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html). You can import the type in python using:

    from sensor_msgs.msg import LaserScan

The ```ranges``` data entry in the ```LaserScan``` message  is an array of the
distances from the lidar sensor to the nearest obstacle. The measurements are
taken at regular intervals, ```angle_increment```, from the angle
```angle_min``` to the angle ```angle_max```.

The rainbow points in this image below are the laser scan as visualized in
```rviz```. The color simply corresponds to the intensity of the scan. In the
simulator this is simply the distance, but on the actual lidar it gives you an
indication of how reflective the object you are scanning is. Note that there is
no data in the quadrant behind the car because on the actual car that area is
occluded.

![The racecar in a cubicle](https://raw.githubusercontent.com/mit-racecar/racecar_simulator/master/media/racecar_simulator_rviz_2.png)

## Steps to Success
How you implement the wall follower is entirely up to you. However implementing
the following may get you started in the right direction:

* __Set up ROS structure__: Set up your wall follower node so that it subscribes to laser messages and publishes steering commands. Make sure you can at least make the racecar move fowards at a constant speed and turning angle before working on your controller.
* __Slice up the scan__: Consider slicing the ```ranges``` data into more useful pieces. A majority of the data won’t be useful to you if you only care about a wall to one side. When you can, try to use [```numpy```](https://docs.scipy.org/doc/numpy-dev/user/quickstart.html) operations rather than for loops in your code. [Multidimensional slicing](https://docs.scipy.org/doc/numpy-1.13.0/reference/arrays.indexing.html) and [broadcasting](https://docs.scipy.org/doc/numpy-1.13.0/user/basics.broadcasting.html) can make your code cleaner and much more efficient. You can turn any array into a ```numpy``` array with [```np.array```](https://docs.scipy.org/doc/numpy-1.13.0/reference/generated/numpy.array.html), or you can integrate it directly with ros like in [this tutorial](http://wiki.ros.org/rospy_tutorials/Tutorials/numpy).
* __Find the wall__: There are many ways to detect a wall in a laser scan. In a perfect world you might be able to detect it using a single sample of the LIDAR data. However with noisy data and uneven surfaces this might not be enough. A [least squares regression](https://en.wikipedia.org/wiki/Simple_linear_regression) is an easy way to account for more noise. The [RANSAC](https://en.wikipedia.org/wiki/Random_sample_consensus) algorithm can “upgrade” an existing model (like least squares) to be more robust to outliers. _Note: Attempt RANSAC only if you've already built a functional wall follower. It might be overkill_
* __Use a PD or PID__: A robust wall follower algorithm that can handle wavy wall contours and corners should probably use some sort of [PD or PID control](https://en.wikipedia.org/wiki/PID_controller#Discrete_implementation). Simple P (proportional) control is often not enough to create a responsive and stable system. Think about the geometry of the wall following problem and how you could use it to get a better approximation of the D (derivative) term than if you used finite differences.

## Starter Code

We have some start code for you in here:

    src/wall_follower.py

However if you want to add more python files to keep your code organized, feel
free to do so.

The template code specifies a couple useful parameters which you can use to make your code more flexible:

You must use the following ROS parameters in your follower:

* ```desired_distance```: The distance in meters the racecar should maintain from the wall
* ```velocity```: The speed the racecar should move in meters per second.
* ```side```: The side the wall is following represented as an integer. +1 represents the left wall and -1 represents the right wall. We chose this convention because typically we will assume the car is pointing in the positive _x_ direction. That means the left side of the car will point to the positive _y_ axis and the right side will point to the negative _y_ axis. 

To run your wall follower launch:

    roslaunch wall_follower wall_follower.launch

<br/>
[Back](./)

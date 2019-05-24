---
title: Setting Up Your Computer
layout: post
---

<!-- ======================================================================= -->

## Overview
We will be working with both the RACECAR hardware and simulator today. Although
basic interactions with the hardware require only
[SSH](https://en.wikipedia.org/wiki/Secure_Shell), the full experience
requires a local installation of
[Robot Operating System (ROS)](http://www.ros.org/) on a GNU/Linux machine.
(Note: Although ROS recently started supporting an
[official Windows 10 build](https://wiki.ros.org/Installation/Windows),
it is new and thus untested with our platform.)

<!-- ======================================================================= -->

<br/>
## What is ROS?
Despite its name, the Robot _Operating System_ is not actually a bona fide OS.
Rather it is a set of robotics middleware built on top of GNU/Linux. ROS is most
commonly used in conjunction with
[Ubuntu](https://www.ubuntu.com/), as ROS releases are tied to Ubuntu releases.
For example:
- Ubuntu 18.04, Bionic Beaver → ROS Melodic Morenia
- Ubuntu 16.04, Xenial Xerus → ROS Kinetic Kame

That being said, [Debian](https://www.debian.org/) is also well supported since
Ubuntu is derived from it.

If you have never used ROS before, it is best thought of as a standardized
[pub/sub messaging protocol](https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern)
with some handy libraries and visualization tools.

<!-- ======================================================================= -->

<br/>
## Setup
The next few sections will walk you through getting your personal machine setup
with ROS as either a native install, Docker install, or Virtual Machine.
* If you have a GNU/Linux machine (especially Ubuntu or Debian) and are
comfortable on it, a native install will give you the best performance.
* If you are on Windows, MacOS, or an unsupported flavor of GNU/Linux, BSD,
etc., we encourage using our new [Docker](https://www.docker.com/) image.
* If you have your own copy of VMWare installed, and prefer VMs to Docker
(why would you though?), we can provide a Debian-based VM preloaded with all
the software you'll need.

_If you already have ROS installed, you're good to go! Just make sure you have
the following ROS packages installed: velodyne, ackermann-msgs, joy, and serial.
Installation instructions for these packages are included at the end of the
Native ROS Install section._

<!-- ======================================================================= -->

<br/>
## Setup (Native ROS - Ubuntu/Debian)
### Step 1: Install ROS
Based on your flavor of GNU/Linux, follow the linked installation instructions
below. Be sure to install the `ros-VERSION-desktop-full` version of ROS.
* [Install ROS on Ubuntu 18.04](https://wiki.ros.org/melodic/Installation/Ubuntu)
* [Install ROS on Ubuntu 16.04](https://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Install ROS on Debian Stretch](https://wiki.ros.org/melodic/Installation/Debian)
* [Install ROS on Debian Jessie](https://wiki.ros.org/kinetic/Installation/Debian)

_There is an experimental ROS installation for Arch Linux. While we love Arch,
we've found the ROS package unreliable. If you must, you can
follow the experimental instructions [here](https://wiki.ros.org/melodic/Installation/ArchLinux)._


### Step 2: Install Additional ROS Packages
After ROS installation completes, install these additional ROS packages:
```sh
# install on Ubuntu 18.04 & Debian Stretch
sudo apt install ros-melodic-velodyne ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-serial

# install on Ubuntu 16.04 & Debian Jessie
sudo apt install ros-kinetic-velodyne ros-kinetic-ackermann-msgs ros-kinetic-joy ros-kinetic-serial
```

### Step 3: Install the racecar code

First make a `racecar_ws`:

    mkdir -p ~/racecar_ws/src

Clone the racecar code:

    cd ~/racecar_ws/src
    git clone https://github.com/mit-racecar/racecar_simulator.git

Make the code:

    cd ~/racecar_ws
    catkin_make
    source devel/setup.bash

<!-- ======================================================================= -->

<br/>
## Setup (Docker)
### Step 1: Install Docker Community Edition (CE)
Based on your OS, follow the linked installation instructions below.
* [Windows](https://docs.docker.com/docker-for-windows/install/)
* [MacOS](https://docs.docker.com/docker-for-mac/install/)
* [Ubuntu](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
* [Debian](https://docs.docker.com/install/linux/docker-ce/debian/)
* [Fedora](https://docs.docker.com/install/linux/docker-ce/fedora/)

### Step 2: Run the Docker Image


Start the docker image by running:

    sudo docker run -ti --net=host racecar/racecar

This will download the docker image the first time it is run and will cache it for future use.

On some operating systems (OS X?) the `--net=host` flag does not properly forward ports. This can be fixed by manually specifying:

    sudo docker run -tip 6080:6080 -p 5900:5900 racecar/racecar

For more instructions on using the docker image see [here](https://github.com/mit-racecar/racecar_docker/blob/master/README.md).

<!-- ======================================================================= -->

<br/>
## Setup (VMWare)
_Unless you have a strong preference for VMs and your own copy of VMWare, we
recommend using the Docker image. Docker CE is free, VMWare is not._

### Step 1: Check VMWare is Installed
Confirm you have VMWare installed on your computer. We do _not_ have product
keys for this.

### Step 2: Download the Image
You can download the RACECAR virtual machine [here](https://www.dropbox.com/s/vf0fv9kc1es3b6e/racecar_2018_02_01.ova).

### Step 3: Import and Run
Import and run the .ova file. This changes based on which copy of VMWare you
personally own.

<!-- ======================================================================= -->

<br/>
[Back](./)

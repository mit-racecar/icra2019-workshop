---
title: RACECAR Hardware
layout: post
---

## Electrical Safety

The racecar runs on relatively low voltage ([≤ 20V](https://www.amazon.com/Energizer-XP18000-Universal-External-Netbooks/dp/B002K8M9HC))
so we are not too concered about dangerous shocks.
But as with any electrical system it is always important to take proper safety
precautions. For example ground yourself before touching any of the exposed
circuit boards like the TX2.

Please have all members of your team read and sign the electrical safety form
here before starting work on the racecar:
https://eecs-ug.scripts.mit.edu:444/safety/index.py/6.141

## The Racecar

### Connections and Power

Once you have your car, search for its number. You can find it in two places;
on top of the car's lidar and on the front of your router. The number will be
in block letter stickers. If you have an older car or router there might be
other numbers written or labeled on it that you can ignore.

![car_number](media/40500596821_e133bedd83_k.jpg)

Plug your router into an outlet and use the ethernet cable to connect it to a
working ethernet port (not all the ports in 32-080 work). Make sure you are
using the **12V power supply** that says "TP-Link" on it. **Using the other
power supply will fry your router**.
Then connect to either of these two wifi networks on your laptop using the
password ```g0_fast!```:

    RACECAR_AP_[YOUR_CAR_NUMBER]
    RACECAR_AP_[YOUR_CAR_NUMBER]_5GHZ

The 5ghz network provides a faster connection but has more limited range.

![router](media/39605336515_5d5459a801_k.jpg)

Check the battery status on your racecar by pressing the power button on your
car's primary battery.
This may be the black energizer pictured below or this
[grey battery](https://www.amazon.com/dp/B07JJTYH8F).
On the hokuyo cars the battery sits right on top of the car.
On the velodyne cars the battery is velcroed under the car (be careful when
pulling it out).
After powered on these batteries will remain on until power stops being drawn
from them so please remember to unplug your power cables when the car is not in
use.

![hokuyo_battery](media/40500597871_792493a139_k.jpg)
![velodyne_battery](media/39604959195_914cb8f59f_k.jpg)

If your battery is low, charge it with the 18V adapter. 
Note that the car will probably turn off when you disconnect the power adapter;
during the switch back to battery power there is a moment where the TX2 does not
have enough power to stay on.
You will need to disconnect the car from the power adapter when you want to
drive it around.
The battery lasts a surprisingly long time, so as long as you keep the battery
charged when you are not working it can last the entire lab.

![energizer_power](media/39791091874_4da61acfd2_k.jpg)

Also charge your motor battery by plugging it into the charger that looks like a
blue block.
Hold the start button for 2 seconds to charge - you should hear the battery fans
begin to spin.
This battery won't last as long, especially when you are going fast, so remember
to charge it when the car is not moving. The TX2 will not be affected if the
motor battery gets unplugged. 

![motor_power](media/39790637494_e1ef9b0292_k.jpg)

Connect the two power cables to the energizer battery.
One powers the lidar and the TX2. 
The other powers the USB hub (which powers the ZED camera and IMU).
If everything is receiving power, you should see LEDs light up on the TX2 and
IMU and you should hear the lidar spinning (listen closely).

![energizer_plugged](media/39604959525_44ff049f74_k.jpg)

Power on the TX2 by pressing the rightmost button on the port side of the car
labeled "power". The button should light up green.

![TX2](media/40500596601_71f9b0ede8_k.jpg)

### SSH

When you're connected to the wifi with the TX2 powered on , you can connect
directly to the car using:

    ssh racecar@192.168.1.[CAR_NUMBER]
        
The password is ```racecar@mit```. If you can't connect make sure you are still
on the correct Wi-Fi network.

The car is running Ubuntu just like the virtual machine.
It should be familiar, but poke around to get comfortable with the structure.
Just like in the simulator, you will often need multiple terminal windows open
in order to launch different ros nodes.
There are many ways to do this through ```ssh```:

- Open multiple windows on your local machine and ```ssh racecar@192.168.1.[CAR_NUMBER]``` in each one of them. You can even ssh from multiple computers at the same time but make sure you are communicating with your team members if you do this.
- Use [screen](https://kb.iu.edu/d/acuy) to open layered windows in terminal and navigate through them with key commands.
- Use ```ssh``` with the ```-X``` flag to enable X11 forwarding. With this flag you can launch graphical programs in the ```ssh``` client and have them displayed on your local machine. For example you could run ```xterm &``` to get a new terminal window. 
- Consider making bash aliases to make these steps easier.

### Manual Navigation

When you are ready, disconnect the power adapters to the energizer and plug the
batteries in.

![motor_plugged](media/39604958785_8e8161b88e_k.jpg)

Turn on the TX2 and recconect to the racecar if necessary.
Get the car to a safe place (_not on a table!_) and launch ```teleop``` just
like in the simulator:

    teleop

Now you should be able to move the car around with the joystick!
**You need press the left bumper before the car can move.**
This is a known as a
[Dead man's switch](https://en.wikipedia.org/wiki/Dead_man%27s_switch) and it is
an easy way to stop the car from crashing - just let go of the trigger.

#### The car isn't moving

- Make sure the joystick is connected and in the right mode by running ~/joystick/test_joystick.sh
- Are you pressing the left bumper on the joystick?
- Make sure the motor battery is plugged in and charged.

### RViz

_If you're using the Docker image, run rviz inside your web browser!_

Because ```rviz``` requires 3D libraries you can't run it straight through SSH.
So you will need ```rviz``` to be connected to the car's ```roscore``` rather
than the one on your local machine.
To do this first edit your ```/etc/hosts``` file (requires ```sudo```) and add
the following line:

    192.168.1.[CAR_NUMBER]     racecar
    
This essentially makes the string ```racecar``` equivalent to the IP of the car.
One benefit of this is that you should now be able to SSH in to the car by
running:

    ssh racecar@racecar
    
Moreover, if your username is racecar (it is in every VM), you don't need to
specify the ```username@```, you can just do:

    ssh racecar
    
Now that you've set up the hostname (you only ever need to do that once), you
can make ```rviz``` listen to the car's ```roscore``` by running the following
command.

    export ROS_MASTER_URI=http://racecar:11311

You also need to set your own IP for 2-way communication by running:

    export ROS_IP=192.168.1.[YOUR_COMPUTER'S_IP]
    
You can find your IP address by running ```hostname -I``` or ```ip addr```.
**If you are on the VM you must set your network adapter to
"Bridged (Autodetect)", otherwise you will not have an IP on the network.** Note
that these commands need to be run in every single terminal that you want to be
connected to the car's roscore, so it is worth considering making an alias for
them or adding them to your ```.bashrc```.

Now if you run ```teleop``` on the car you should be able to open up ```rviz```
and visualize the real lidar data (topic ```/scan```) and the IMU data
(```/imu/data```).

### Cleaning Up

Before you get too far ahead, remember that when you are done using the racecar,
you **must unplug all power cables**. This includes 2 cables that connect to the
energizer battery and the motor battery. Not doing this can destroy the
batteries and the servo.

![motor_unplugged](media/39604958985_bd32f3ea16_k.jpg)
![energizer_unplugged](media/39791091494_1fee2d09a0_k.jpg)

<br/>
[Back](./lab-wall-follow-hardware)

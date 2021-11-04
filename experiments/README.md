# SpheroJameoba

SpheroJameoba is a python project to use the Orbotix Sphero Mini in a swarm with ROS for the Jameoba project.

## Table of content

1. [How to get started](#how-to-get-started)
2. [File descriptions](#file-descriptions)
    - [SingleControlNode.py](#singlecontrolnodepy)
    - [MultiControlNode.py](#multicontrolnodepy)
    - [TargetTrackNode.py](#targettracknodepy)
    - [AprilTagNode.py](#apriltagnodepy)
3. [Closed loop control examples](#closed-loop-control-examples)
    - [1 - Apriltag as the target for a single Sphero](#1---apriltag-as-the-target-for-a-single-sphero)
    - [2 - Apriltag as the target for N Spheros](#2---apriltag-as-the-target-for-n-spheros)
    - [3 - Apriltag as the target for N Spheros in formation](#3---apriltag-as-the-target-for-n-spheros-in-formation)
    - [4 - Apriltag as the target for N Spheros on multiple machines](#4---apriltag-as-the-target-for-n-spheros-on-multiple-machines)
4. [How to setup and use your multiple machine network](#how-to-setup-and-use-your-multiple-machine-network)
5. [How to find your Sphero MAC address](#how-to-find-your-sphero-mac-address)
6. [Raspberry Pi 3B+ setup](#old-files)

## How to get started

Clone this repository in a directory. In the directory, open a terminal and initialise the submodules :

```shell script
$ git submodule init
$ git submodule update
```

Go to the apriltags3py directory and initialize his submodule too :

```shell script
$ cd apriltags3py 
$ git submodule init
$ git submodule update
```

Go to the apriltags directory and build the library :

```shell script
$ cd apriltags
$ cmake .
$ make
```

If everything went right, you should be able to use the package.

*** ATTENTION : If you are on a computer that won't run the AprilTag tracker (don't use the camera), building this library is not necessary.

Before running any file, you should install the required python packages by using :

```shell script
$ pip3 install -r requirements.txt
```

*** ATTENTION : If pip3 is not installed on your computer, run this line to install it :
```shell script
$ sudo apt install python3-pip
```

If you need to run GUI Applications, make sure to install `Tkinter` by using : 

```shell script
$ sudo apt install python3-tk
```

## File descriptions

Here is how each files can be used and their different options.

### SingleControlNode.py

To run this scripts, run this line in a terminal : 

```shell script
$ python SingleControlNode.py X -i -s -b -c R G B -g P I D
```

where:
 
 1) `X` is the tag id of the Sphero you want to control.
 2) `-i` is optional. Put this tag to reset the heading of the Sphero.
 3) `-s` is optional. Put this tag to run this as a simulation (no BlueTooth connections).
 4) `-b` is optional. Put this tag if
    you are using a Sphero Bolt.
 5) `-c R G B` is optional. Put this tag to set the color of the Sphero. r, g and b are the RGB value for the color.
 6) `-g P I D` is optional. Put this tag to set the gain of the PID controller.

### MultiControlNode.py

To run this scripts, run this line in a terminal : 

```shell script
$ python MultiControlNode.py X1 X2 X3 -i -s -b -c R G B -g P I D
```

where:
 
 1) `Xn` is the tags ids of the Sphero you want to control. There is 3 tags here but you can have from 1 to 4 tag ids.
 2) `-i` is optional. Put this tag to reset the heading of the Spheros.
 3) `-s` is optional. Put this tag to run this as a simulation (no BlueTooth connections).
 4) `-b` is optional. Put this tag if you are using Sphero Bolt.
 5) `-c R G B` is optional. Put this tag to set the color of the Sphero. r, g and b are the RGB value for the color.
 6) `-g P I D` is optional. Put this tag to set the gain of the PID controller.

### TargetTrackNode.py

To run this scripts, run this line in a terminal : 

```shell script
$ python TargetTrackNode.py X -f F -r R
```

where:
 
 1) `X` is the tag id of the target.
 2) `-f F` is optional. Use this tag to change the formation mode. If not used, the default value is used.
 3) `-r R` is optional. Use this tag to specify the radius of the circle formation or the distance between robots in cm.

Here are the formation options :

| Mode (F) | Description                                         | Default |
|:--------:|:----------------------------------------------------|:-------:|
| "None"   | Every Sphero will try to get on target              | X       |
| "Around" | The Spheros will circle the target                  |         |
| "LineXP" | The Spheros will make a line on the positive X axis |         |
| "LineXN" | The Spheros will make a line on the negative X axis |         |
| "LineYP" | The Spheros will make a line on the positive Y axis |         |
| "LineYN" | The Spheros will make a line on the negative Y axis |         |

### AprilTagNode.py

To run this scripts, run this line in a terminal : 

```shell script
$ python AprilTagNode.py X -u -s -f F
```

where:
 
 1) `X` is the number of tags you want to track.
 2) `-u` is optional. Put this tag to show/save the unwarped image.
 3) `-s` is optional. Put this tag to save the video and the data.
 4) `-f F` is optionnal. Use it to specify the max update frequency for the tag detection.

### LeaderFollowerControlNode.py

To run this scripts, run this line in a terminal : 

```shell script
$ python LeaderFollowerControlNode.py X1 X2 X3 -i -k K -c R G B -g P I D
```

where:
 
 1) `Xn` is the tags ids of the Sphero you want to control. There is 3 tags here but you can have from 1 to 4 tag ids.
 2) `-i` is optional. Put this tag to reset the heading of the Spheros.
 3) `-k` is optional. Put this tag if you are using Sphero Bolt.
 4) `-c R G B` is optional. Put this tag to set the color of the Sphero. r, g and b are the RGB value for the color.
 5) `-g P I D` is optional. Put this tag to set the gain of the PID controller.

## Closed loop control examples

The following codes use ROS. Therefore, `roscore` need to be running for the following demos to work.

### 1 - Apriltag as the target for a single Sphero

For this demo, you need to run three python scripts: `SingleControlNode.py`, `TargetTrackNode.py` and `AprilTagNode.py`.
Run those three lines in a their own terminal :

```shell script
$ python SingleControlNode.py -i 0
$ python TargetTrackNode.py 1
$ python AprilTagNode.py 2
```

### 2 - Apriltag as the target for N Spheros

For this demo, you need to run three python scripts: `MultiControlNode.py`, `TargetTrackNode.py` and `AprilTagNode.py`.
Run those three lines in a their own terminal :

```shell script
$ python MultiControlNode.py -i X1 X2 ... Xn
$ python TargetTrackNode.py N
$ python AprilTagNode.py N+1
```

where N is the number of Spheros you want to control.

### 3 - Apriltag as the target for N Spheros in formation

For this demo, you need to run three python scripts: `MultiControlNode.py`, `TargetTrackNode.py` and `AprilTagNode.py`.
Run those three lines in a their own terminal :

```shell script
$ python MultiControlNode.py -i X1 X2 ... Xn
$ python TargetTrackNode.py -f "Around" N
$ python AprilTagNode.py N+1
```

where N is the number of Spheros you want to control.

### 4 - Apriltag as the target for N Spheros on multiple machines

For this demo, you need to run three python scripts: `MultiControlNode.py`, `TargetTrackNode.py` and `AprilTagNode.py`.
Run those three lines in a their own terminal :

> Main machine
```shell script
$ python TargetTrackNode.py -f "Around" N
$ python AprilTagNode.py N+1
$ python MultiControlNode.py -i X1 X2 ... X5
```
> Raspberry Pi
```shell script
$ python MultiControlNode.py -i X6 X7 ... Xn
```

where N is the number of Spheros you want to control.

## How to setup and use your multiple machine network

To set up your multiple machine network make sure :

- Every computer/Raspberry Pi should be on the same network
- On every Raspberry Pi, add this line to your ~./bashrc
```shell script
export ROS_MASTER_URI=http://x.x.x.x:11311
# where x.x.x.x is the local IP address of the master computer.
```

- You need to run `roscore` on the master computer only.
- Use `ssh` to control the Raspberry Pi from the master computer. To do it, use this line :
```shell script
$ ssh spenko@x.x.x.x
# where x.x.x.x is the local IP address of the Raspberry Pi.
```

As of November 28th 2019, the IP addresses used are :

| Computer                | IP address   | ROS role |
|:------------------------|:-------------|:---------|
| spenko@robotics-desktop | 192.168.2.7  | Master   |
| spenko@robotics-pi1     | 192.168.2.10 | Slave    |
| spenko@robotics-pi2     | 192.168.2.11 | Slave    |
| spenko@robotics-pi3     | 192.168.2.12 | Slave    |

## How to find your Sphero MAC address
 
For the known Spheros, the addresses are available in the `Jameoba/Spheros.py MAC_ADDRESS` list. 
Use the tag id of the Sphero as an index to get the corresponding MAC

On linux, you can use `sudo hcitool lescan` to list the available device's MAC address.

## Raspberry Pi 3B+ setup

For a complete guide on how to setup your Raspberry Pi 3B+ to work with this project, 
see  the [the Raspberry Pi configuration tutorial file](RPI_CONFIG.md) [(RPI_CONFIG.md)](RPI_CONFIG.md)

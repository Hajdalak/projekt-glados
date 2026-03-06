---
title: Robolab LAR support package
author: Libor Wagner <libor.wagner@cvut.cz>
date: 2021-02-10
update: 2023-03-16
---

# LAR

 - [FAQ](doc/faq.md) - questions, problems

## Get Started

This is the basic workflow with Turtlebots. Assuming that one works from lab using either own personal linux computer of one of the lab computers.

### Step 1: Connect to the robot

 - First you have to connect to the robot using ssh.
 - If you are using your own computer it must be connected to lab wifi or you can use server turtle.felk.cvut.cz.
   For detail see [Lab Computers and Wifi](./doc/lab_computers_and_wifi.md).

```sh
# from lab computer
ssh -X [username]@[turtleName]

# from personal linux computer connected to lab wifi
ssh -X [username]@[turtleIP]

# for example
ssh -X ros@turtle01
```

 - More info in [Lab Computers and Wifi](./doc/lab_computers_and_wifi.md)

### Step 2: Enter singularity container

 - ROS and the Turtlebot drivers are installed as Singularity container, in order to work with the robot one must enter the container using following command:

```sh
# mount local partition, this might result in error if already mounted
mount /local

# start singularity
singularity shell /local/robolab_noetic_amd64.simg
```

 - More info in [Singularity Containers](./doc/singularity_container.md)

### Step 3: Source ROS environment

 - At this point we will need multiple terminals, one for the robot driver, and at least one to start your program and other tools. This can be done by either go through steps 1 to 3 again in new terminal or use a terminal multiplexer, such as tmux (see [doc/tmux.md](./doc/tmux.md)).
 - In order to use ROS and Turtlebot, you have to source ROS environment:

```sh
source /opt/ros/lar/setup.bash
```

### Step 4: Start robot driver

 - The Turtlebot driver is a collection of ROS nodes which provide all the functionality of the robot.
 - To start in run the following command:

```sh
roslaunch robolab_turtlebot bringup_realsense_D435.launch
```

 - There will be probably bunch of errors which should be ignored if everything else works.
 - The driver will run inside this terminal, to stop the driver press `ctrl-c` in the drivers terminal.

### Step 5: Start your application

 - This step is to be done in **new terminal** (either with `tmux` of by going throu steps 1 to 3 once more).
 - In `b3b33lar` we will use python as a programming language and all the ROS stuff is hidden under [Turtlebot](https://gitlab.fel.cvut.cz/robolab/robolab_turtlebot)
 - The robot application is just a python script, same as one of the demonstration scripts which you can try:
 - **Bumper test**: simple script that will subscribe to bumber events and prints them when the bumper is activated.

```sh
python3 /opt/ros/lar/share/robolab_turtlebot/scripts/bumper_test.py
```

  - **Move 1m**: moves the robot approximately 1m in forward direction.

```sh
python3 /opt/ros/lar/share/robolab_turtlebot/scripts/example_move_1m.py
```

 - **Random Walk**: drive the robot forward until obstacle is detected then rotate in random direction until obstacle disappears, then repeat. This script will also whow the depth map on a screen so it will only work when you connected with the `-X` flag.

```sh
python3 /opt/ros/lar/share/robolab_turtlebot/scripts/random_walk.py
```

## Copy files to robot

 - Use your home directories which are synchronized across all computers including robots.

 - Or use scp from linux of linux-subsystem on windows

```sh
scp file.py [username]@[turtleIP]:~/file.py
```

 - Or use [WinSCP](https://winscp.net/eng/index.php)
 - Or use [git](https://git-scm.com/) and [gitlab](gitlab.fel.cvut.cz)

## Copy ssh id (aka stop writing password all the time)

 - First you  need to generate ssh id (skip if you have it already)
 - When asked leave password empty so you do not have to write it every time for ssh this time (but acknowledge that it is security risk)
```sh
ssh-keygen
```

 - Copy id to the turlebot, is needs to be done only once as the home directories are shared between all turtlebots.
```sh
# copy ID into the turtlebot
ssh-copy-id [username]@[turtleIP]
```

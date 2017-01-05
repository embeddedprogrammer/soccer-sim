Soccer Simulator
================

Based off of the work from Jacob White, TA. (See [walle repo](https://github.com/embeddedprogrammer/soccer/tree/gazebo)).

**See it in action in the Chicken McThuggets [code](https://github.com/zamora18/robot-soccer/tree/develop/ros/src/soccersim)!**

We added the ability to use our ROS package with the Gazebo physics simulator, without having to move any code. We simply create a `roslaunch` file that remaps topics between our code and the simulator so that the correct items are being sent.

We also added the ability to click on the vision screen and place the ball, or click and drag to drag the ball wherever we needed to.

A naive kicker was also implemented to help us simulate our actual robot.

See an demo of the simulator [here](https://youtu.be/bMZNFNGh_Pk).

#### Setting Up Gazebo ####

Assuming you installed `ros-*-desktop-full`, you should already have Gazebo installed. Now, you just need to make sure you have some ROS packages installed that allow Gazebo and ROS to be friends.

```bash
sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control
```
(See [Installing gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros_installing) for more)

You also need to make sure that you install the `mesa-utils` package. This package is an open source OpenGL implementation.

```bash
sudo apt-get install mesa-utils
```

You can run `glxgears` to test your 3D graphics. If you have problems, you will need to update your graphics card. If you have an Nvidia graphics card, this will probably be easier, since that's all we ever dealt with..... see below.

#### Problem with Downloading Models ####

With Indigo and Gazebo 2.2, there are problems with downloading models. See [this](http://answers.ros.org/question/199401/problem-with-indigo-and-gazebo-22/?answer=199475#post-id-199475) ROS answer for more info. Download the models from the bitbucket link and then move the models to your `~/.gazebo/models/` directory.

#### Problems with Graphics ####

Often, you may see that Gazebo comes up with black screen (you can minimize it and then open it and it may fix it). Or maybe `glxgears` runs with strange artifacts. Often Linux problems that have to do with 3D rendering come from not using the correct graphics drivers.

You can use `lspci` to see what your graphcis hardware is. You can use `sudo lspci -vvv` to see what the current kernel driver is for your graphics card. There is a linux open source driver called `nouveau` that is probably being used. Go figure out how to install your graphics driver based on your card, it's product number, and Google.

#### Ignoring This Package on the ODROID ####

In order to ignore this package in an environment where `ros-*-desktop-full` is not installed (i.e., no Gazebo), use the `CATKIN_IGNORE` file by doing the following:

```bash
touch robot-soccer/ros/src/soccersim/CATKIN_IGNORE
```

This file is ignored by git. By simply creating it, `catkin_make` will no longer try to build it.

#### Features Added ####

- Changed ball friction and size
- Changed field size
- Click on `Overhead Vision Camera` view to place ball
- Allow connection to actual ROS code so that everything can be simulated, not just AI
- Allow rostopic remapping on most topics/services
- Kicker
- Camera speed changed to 30fps
- Vision scaling properly between simulation-world and Gazebo-world

#### Features Wish-list ####

- Be able to move robots by clicking as well as ball
- Add a circle to middle of field so that vision code can dynamically size everything

See the branch `sim_move_bots` for the code.

#### How to Run ####

See the `scripts/simulator_setup.bash` file to understand more about the suite of commands that can be used to control the simulator.

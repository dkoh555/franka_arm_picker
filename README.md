# ME495 Embedded Systems Homework 3
Author: Damien Koh

## Repository Description
This repository contains two packages, `diff_drive` and `pickmeup`, to demonstrate the use of Gazebo and my group's own MoveIt class respectively.

For the `pickmeup` package to be used properly, our group's `motion_plan_pkg` will be required. More detailed setup instructions will be detailed below.

### Setup
1. Ensure that you have the `diff_drive`, `pickmeup`, and `motion_plan_pkg` packages installed by using the `vcs` tool (https://github.com/dirk-thomas/vcstool) to import the additional repos.
    1. Ensure the `vcs` is setup on your computer
    2. In your `ws/src` directory, call run `vcs import < mover.repos`
    3. Build your workspace
2. Make sure that your current workspace as well as the workspace containing the Franka Panda arm code are both sourced. (This is not required if only using the  `diff_drive` package)

## Package: pickmeup

### Quickstart
1. To have the real Franka Panda arm pick up an object right in front of it, run `ros2 launch pickmeup pickup.launch.xml fake_mode:=false`
2. To have the virtual Franka Panda arm make motions imitating picking up an object right in front of it, run `ros2 launch pickmeup pickup.launch.xml fake_mode:=false`

### Additional Commands
Whether you want to run the virtual or real Franka Panda arm, feel free to input custom coordinates to either launch file to pick up the object at a desired location.

Here are some examples:
1. For the real arm, run `ros2 launch pickmeup pickup.launch.xml fake_mode:=false x:=0.3 y:=0.55`
2. For the fake arm, run `ros2 launch pickmeup pickup.launch.xml x:=0.45 y:=-0.3 z:=0.3`

### Videos

1. Here is a video demonstrating the `pickmeup` package working on a real Franka Panda arm:

https://github.com/ME495-EmbeddedSystems/homework3group-dkoh555/assets/107823507/accaa9c1-f210-46bf-8ed7-174e0c581174

2. Here is a video demonstrating the `pickmeup` package working on a virtual Franka Panda in rviz:

[arm_fake_mode.webm](https://github.com/ME495-EmbeddedSystems/homework3group-dkoh555/assets/107823507/6ca3d120-c343-4d1e-8788-496cd8be1b8e)


## Package: diff_drive

### Quickstart
1. Use `ros2 launch diff_drive ddrive_rviz.launch.xml` to view the diff_drive robot in rviz.
2. Use `ros2 launch diff_drive ddrive.launch.xml` to simulate the diff_drive robot flipping in Gazebo and view it in rviz with the appropriate configuration.

### Videos

1. Here is a video of the flipping diff_drive simulation running in Gazebo:

[flip_gazebo.webm](https://github.com/ME495-EmbeddedSystems/homework3group-dkoh555/assets/107823507/e3e66b39-a0ae-4b3d-9262-d0ebbe80d854)
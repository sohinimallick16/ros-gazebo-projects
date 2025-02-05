# Go Chase It

## Project Overview

In this project, I have built a ball chaser mobile robot using ROS (Robot Operating System). The project is divided into two major tasks:

1. **Design the Robot**: Create a mobile robot and house it inside a Gazebo world.
2. **Program the Robot**: Write ROS nodes to make the robot chase white-colored balls through the world.

## Project Structure

The project includes the following ROS packages and directories:

- `my_robot/`
  - Contains the robot design and Gazebo world files.
- `ball_chaser/`
  - Contains the C++ ROS nodes for chasing the ball.

## Requirements

### Robot Design

- Designed a differential drive robot using the Unified Robot Description Format (URDF). The robot includes:
  - Lidar and camera sensors.
  - Gazebo plugins for the robot’s differential drive, lidar, and camera.
  - Housed inside the Gazebo world.
- Significant changes from the sample taught in the project lesson:
  - Color
  - Wheel radius
  - Chassis dimensions
- The robot design is stable.

### Gazebo World

- The `my_robot` ROS package contains the Gazebo world.
- The Gazebo world includes:
  - A new world designed using the building editor for this project.
  - A white-colored ball.

### Ball Chasing

- The `ball_chaser` ROS package contains two C++ ROS nodes to interact with the robot and make it chase a white-colored ball:
  - **drive_bot**:
    - A `ball_chaser/command_robot` service.
    - The service accepts linear x and angular z velocities.
    - The service publishes to the wheel joints.
    - The service returns the requested velocities.
  - **process_image**:
    - Subscribes to the robot’s camera image.
    - A function to analyze the image and determine the presence and position of a white ball.
    - Requests a service to drive the robot towards a white ball (when present).

### Launch Files

- The submission includes `world.launch` and `ball_chaser.launch` files that launch all the nodes in this project:
  - **world.launch**:
    - Launches the world (which includes a white ball).
    - Launches the robot.
  - **ball_chaser.launch**:
    - Runs the `drive_bot` C++ node.
    - Runs the `process_image` C++ node.

## Installation and Usage

1. Clone the repository to your local machine.
2. Navigate to the project directory and build the packages using `catkin_make`:
    ```sh
    cd /path_to_your_catkin_workspace
    catkin_make
    source devel/setup.bash
    ```
3. Launch the Gazebo world:
    ```sh
    roslaunch my_robot world.launch
    ```
4. In a new terminal, launch the ball chaser nodes:
    ```sh
    roslaunch ball_chaser ball_chaser.launch
    ```

## Conclusion

This project demonstrates my ability to design a mobile robot, create a simulation environment, and program ROS nodes to interact with the robot. 
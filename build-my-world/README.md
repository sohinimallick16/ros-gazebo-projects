# Build My World

## Project Overview

The objective for this project was to design and build a simulated robotic environment using Gazebo. The project involved creating a Gazebo world, designing and importing models, and writing a C++ plugin to interact with the environment. 

## Project Structure

The project includes the following directories and files:

- `world/`
  - Contains the Gazebo world file.
- `model/`
  - Contains the building and robot model files.
- `script/`
  - Contains the C++ plugin code.
- `CMakeLists.txt`
  - The CMake configuration file for building the plugin.

## Requirements

- Designed a building using Gazebo's Building Editor and stored it in the `model` directory. The building meets the following requirements:
  - Different from the one shown in the sample simulation world of the course.
  - Single floor.
  - Enough space for robots to navigate.
  - Includes at least one feature.
  - Includes at least one color.

- Designed a robot using Gazebo's Model Editor and stored it in the `model` directory. The robot meets the following requirements:
  - Different from the one shown in the sample simulation world.
  - Robot links are connected through joints.

- Created a Gazebo world and stored it in the `world` directory. The world meets the following requirements:
  - Different from the one shown in the sample simulation world.
  - Contains the building model (e.g., house, restaurant, etc.).
  - Contains two instances of the robot model.
  - Contains one model from the Gazebo online library.

- Created a C++ plugin and stored it in the `script` directory. The CMakeLists.txt file is stored in the main project directory. The plugin meets the following requirements:
  - Prints “Welcome to Sohini’s World!” message.

## Installation and Usage

1. Clone the repository to your local machine.
2. Navigate to the project directory.
3. Build the project using CMake:
    ```sh
    mkdir build
    cd build
    cmake ..
    make
    ```
4. Launch the Gazebo world:
    ```sh
    gazebo world/Office.world
    ```

  ## Conclusion

This project demonstrates my ability to create and interact with a Gazebo simulation environment, showcasing skills in model design, simulation world creation, and plugin development. 
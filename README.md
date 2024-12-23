<!-- PROJECT LOGO -->
<div align="left">
    <img src="docs/media/Banner.png" alt="Banner" >
</div>
 
 # robotics-project-1

<a id="readme-top"></a>


  
 

<h3 align="left">Controlling a robot in an evironment with ROS2 and Gazebo Ignition</h3>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#environments">Our environments</a></li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#Participants">Participants</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

Final project for the Mobile Robots course at UIB as part of the Master's program in Intelligent Systems. The project focuses on the simulation, control, and modeling of a robotic system designed to navigate and interact in a simulated restaurant environment.




### Built With
* [![Python 3][Python-badge]][Python-url]
* [![ROS 2][ROS2-badge]][ROS2-url]
* [![Navigation 2][Nav2-badge]][Nav2-url]
* [![TurtleBot3][TurtleBot3-badge]][TurtleBot3-url]
* [![Gazebo Ignition][Gazebo-badge]][Gazebo-url]







<!-- GETTING STARTED -->
## Getting Started

These are the instructions on setting up this project locally. To get a local copy up and running follow these steps:

### Prerequisites

Use Ubuntu 22.04

- Install ROS Humble (more information: [Install ROS Humble on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))
  
    ```bash
    # Install dependencies
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    
    # Add ROS 2 repository and key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Update apt repository and upgrade packages
    sudo apt update
    sudo apt upgrade
    
    # Install ROS Humble Desktop
    sudo apt install ros-humble-desktop
    ```
    
  Add ROS installation to your bash file:
  ```bash
  # Open bashrc file
  gedit ~/.bashrc
  ```
  
  Add the following line at the end of the file
  ```bash
  source /opt/ros/humble/setup.bash
  ```
  
  Restart your terminal.
    
- Install Navigation2 and Turtlebot3 packages:
    ```bash
    # Install Navigation2 package
    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
    
    # Install Turtlebot3 package
    sudo apt install ros-humble-turtlebot3*
    ```
    
- Install Gazebo Ignition:
    ```bash
    # Get Gazebo key
    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    
    # Install Gazebo
    sudo apt-get update
    sudo apt-get install gz-ionic

    # Install ROS Ignition Bridge for Gazebo Integration
    sudo apt install ros-humble-ros-ign-gazebo
    sudo apt install ros-humble-ros-ign-bridge
    ```
- Install more necessary packages:
  ```bash
  # Install necessary packages
  sudo apt install ros-humble-xacro
  sudo apt install python3-colcon-common-extensions
  sudo apt install python3-rosdep2
  ```
  Initialize rosdep and update
  ```bash
  sudo rosdep init
  rosdep update
  ```



### Installation waiter robot

1. Create ROS workspace folder if it doesn't exist yet
    ```bash
    # Create workspace folder with src folder
    mkdir ros2_ws/src
    # Navigate to src folder
    cd ros2_ws/src
     ```
2. Clone the repo
   ```bash
   git clone https://github.com/mirabud/robotics-project-1.git
   ```
3. Go into the ros2_ws folder and update Ros dependencies
   ```bash
   # update dependencies
   rosdep update
   
   # install dependencies
   rosdep install --from-paths src --ignore-src -r -y
   ```
5. Build the project
   ```bash
   colcon build --symlink-install
   ```
6. Source the package by adding the install.bash file to the bash
   ```bash
   # Open bashrc file
   gedit ~/.bashrc
   ```
  
   Add the following  two lines at the end of the file
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
   Set parameter for the robot model you want to use. There are three different models `waffle`, `burger` and `marses`:

   E.g.
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ```
   Set parameter for which world you want to use. The `world_restaurantV2` is the newest but also with high computational cost. For better performance use `world_restaurant` or `wordl_only`:

   E.g.
   ```bash
   export WORLD=world_restaurantV2
   ```

   
   Restart your terminal.
## Environments
<table>
    <tr>
         <td>world_restaurantV2</td>
         <td>world_restaurant</td>
         <td>world_only</td>
    </tr>
    <tr>
        <td><img src="docs/media/Screenshot from 2024-11-29 02-35-15.png" alt="Banner"></td>
        <td><img src="docs/media/world_restaurant.png" alt="Another Image"></td>
        <td><img src="docs/media/world_only.png" alt="Another Image"></td>
    </tr>
</table>


<!-- USAGE EXAMPLES -->
## Usage
* For restaurant environment:
  ```bash
   ros2 launch turtlebot3 restaurant.launch.py
   ```
    ### Controlling the robot
  1. Using keyboard: 
  
     Open a new terminal and start the teleop module from the turtlebot3 package
     ```bash
       ros2 run turtlebot3_teleop teleop_keyboard
     ```
  2. Move to a point:
     
     In RViz you can set navigation goals by clicking on `Nav2 Goal` and select an arbitrary point on the map.
  3. Waypoint Following:
  
     In RViz you can click on `Waypoint / Nav Through Poses Mode`. Set multiple `Nav2 Goals` like in ii. and then click on `Start Waypoint Following`

  4. Creating a map with cartographer module:

      Start cartographer
     ```bash
     ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
     ```
     Move around with the robot e.g. with `turtlebot3_teleop` until you get a satisfying result. To save the map you can use the following command:
     ```bash
     ros2 run nav2_map_server map_saver_cli -f my_map
     ```
     
     
   ### Change Controller Type
  The DWB_Controller from the nav2 package is set per default as our path controller. If you want to change to a pose controller, the parameter file from the current selected robot needs to be adjusted. As per default nav2 works with a global planner, so we have to change the global planner in the `Planner Server` section and the controller in the `Controller Server` under `FollowPath` section:

  
  <table>
    <tr>
        <td><img src="docs/media/Custom_Global_Planner.png" alt="Banner"></td>
        <td><img src="docs/media/Custom_Pose_Controller.png" alt="Another Image"></td>
    </tr>
</table>



<!-- CONTACT -->


<!-- MARKDOWN LINKS & IMAGES -->
[ROS2-badge]: https://img.shields.io/badge/ROS%202-5A1B28?style=for-the-badge&logo=ros&logoColor=white
[ROS2-url]: https://index.ros.org/doc/ros2/
[TurtleBot3-badge]: https://img.shields.io/badge/TurtleBot3-009CDE?style=for-the-badge&logo=turtlebot3&logoColor=white
[TurtleBot3-url]: https://www.turtlebot.com/
[Nav2-badge]: https://img.shields.io/badge/Nav2-0088CE?style=for-the-badge&logo=ros&logoColor=white
[Nav2-url]: https://navigation.ros.org/
[Gazebo-badge]: https://img.shields.io/badge/Gazebo%20Ignition-2F5A5B?style=for-the-badge&logo=ignition&logoColor=white
[Gazebo-url]: https://ignitionrobotics.org/
[Python-badge]: https://img.shields.io/badge/Python-3.8%2B-3776AB?style=for-the-badge&logo=python&logoColor=white
[Python-url]: https://www.python.org/

## Participants
- [Mira Budenova](https://github.com/mirabud)
- [Alejandro Cedillo Gámez](https://github.com/alexcega)
- [Micha Fauth](https://github.com/michafauth99)
- [Lili Even](https://github.com/Lili00E)
- [Juan Juancho](https://github.com/Juan-c3po)

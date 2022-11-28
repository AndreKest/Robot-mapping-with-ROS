# Robot-mapping-with-ROS
Robot mapping with ROS and Gazebo

Was build for a project at my university.

Map a enivronemnt from Gazebo with a Turtlebot3 robot in Python and publish the map to RVIZ.

------------------------------------------------------------------------------------------
### Folder structure
.<br>
├── build                   &emsp;# Compiled folder with colcon<br>
├── install                 &emsp;# setup data<br>
├── launch                  &emsp;# launch files<br>
├── log                     &emsp;# Log-files from ROS2<br>
├── src                     &emsp;# Own Paket (here: py_mapping)<br>
└── readme.md<br>


------------------------------------------------------------------------------------------
### Versionen:

Python: 3.10.6<br>
ROS2:   ROS-Humble<br>
RVIZ2:  11.2.3<br>
GAZEBO: 11.10.2<br>


------------------------------------------------------------------------------------------
### Installieren ROS2 und Pakete

sudo apt install ros-humble-desktop-full ros-humble-gazebo-* ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* ros-humble-dynamixel-sdk python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update


------------------------------------------------------------------------------------------
### Start project
cd ros2_ws/                           &emsp;&emsp;# go to folder this folder<br>
build colcon                          &emsp;&emsp;# build project<br>
. install/setup.bash                  &emsp;&emsp;# start setup<br>
export TURTLEBOT3_MODEL=burger        &emsp;&emsp;# add robot for simulation<br>
cd launch/                            &emsp;&emsp;# go to launch folder ./launch<br>
ros2 launch mapping_house.launch.py   &emsp;&emsp;# start project | RVIZ2, GAZEBO, Controlling and py_mapping <br>

------------------------------------------------------------------------------------------

# Robot-mapping-with-ROS
Robot mapping with ROS and Gazebo

Was build for a project at my university.

------------------------------------------------------------------------------------------
### Folder structure
.
├── build                   # Compiled folder with colcon
├── install                 # setup data
├── launch                  # launch files
├── log                     # Log-files from ROS2
├── src                     # Own Paket (here: py_mapping)
└── readme.md


------------------------------------------------------------------------------------------
### Versionen:

Python: 3.10.6
ROS2:   ROS-Humble
RVIZ2:  11.2.3
GAZEBO: 11.10.2


------------------------------------------------------------------------------------------
### Installieren ROS2 und Pakete

sudo apt install ros-humble-desktop-full ros-humble-gazebo-* ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* ros-humble-dynamixel-sdk python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update


------------------------------------------------------------------------------------------
### Start project
cd ros2_ws/                           # go to folder this folder
build colcon                          # build project
. install/setup.bash                  # start setup

cd launch/                            # go to launch folder ./launch
ros2 launch mapping_house.launch.py   # start project
                                      # RVIZ2, GAZEBO, Controlling and py_mapping 

------------------------------------------------------------------------------------------

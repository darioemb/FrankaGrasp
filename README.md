# FrankaGrasp
Grasp algorithms on Franka robot

## How to integrate vrep in ros
- Install dependencies `sudo apt install xsltproc`
- export vrep directory executable `export VREP_ROOT=/path/to/vrep_directory`
- create a directory for the plugin where you want `mkdir -p ros_plugin/src`
- `cd ros_plugin/src`
- clone rosInterface plugin for vrep `git clone --recursive https://github.com/CoppeliaRobotics/v_repExtRosInterface.git vrep_ros_interface`
- `cd ..`
- `catkin build`
- `cd $VREP_ROOT`
- `rm libv_repExtRosInterface.so`
- `ln -s /path/to/vrep_ros_interface/devel/lib/libv_repExtRosInterface.so`
- Launch roscore and then launch vrep `./vrep.sh`
- Check if RosInterface is loaded!
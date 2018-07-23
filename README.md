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

## Compile
- Launch `configure.sh` script
- If the `libfranka` compilation shows eorrors then use `git clone --recursive` option

## Launch simulation
- Open 2 terminals and source the setup: `source devel/setup.bash`
- Launch `roslaunch agile_grasp_panda launch_all.launch` and press play on vrep simulator
- In an other terminal launch `rosrun pcl_ros pcd_to_pointcloud [path_to_/file.pcd] cloud_pcd:=/camera/depth_registered/points _frame_id:=/camera_rgb_optical_frame` to load pcd file
- Then to move from central to left piolo use 
```
rostopic pub --once /from_to std_msgs/Int16MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0,1]"
```

You can follow the end-effector position from `/vrep_interface/ee_position` topic or the place error from `/piolo_shift/error` topic or the object position from `/pose_obj` topic.

# Getting started:

### Franka CID:
- `libfranka`
- `franka_ros`

`franka_ros` is only required if you want to control your robot using ROS.
 
### Installing from the ROS repositories:
`sudo apt install ros-kinetic-libfranka ros-kinetic-franka-ros`
for check installed pkg:
`dpkg --get-selections | grep "pattern"`
downloader the source code:
`git clone --recursive  https://github.com/frankaemika/libfranka`
build lib (in external folder of ros_ws):
```
cd libfranka
mkdri build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

### Building the ROS pkg:
```
cd "path to desider folder"
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/kinetic/setup.sh
catkin_init_workspace src
```
install any missing dep and build pkg:
```rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
source devel/setup.sh
```

### Haf Grasping:
to existing "src" folder of ROS_ws:
`cd "ROS_ws"/src`
download code:
```
git clone -b kinetic https://github.com/davidfischinger/haf_grasping.git
cd haf_grasping/libsvm-3.12
make
cd "ROS_ws"
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.sh
roslaunch haf_grasping haf_grasping_all.launch
rostopic pub /haf_grasping/input_pcd_rcs_path std_msgs/String "$(rospack find haf_grasping)/data/pcd2.pcd" -1
```

###haf_pick_place pkg ROS:
```
$1: roslaunch prova vrep.launch
$2: roslaunch haf_grasping haf_grasping_all.launch 
$3: rosrun prova haf_pick_place 
$4: rostopic pub /haf_grasping/input_pcd_rcs_path std_msgs/String "$(rospack find haf_grasping)/data/ring12.pcd" -1
$5: rostopic pub --once /from_to std_msgs/Int16MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0 , 1]"
```


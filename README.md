# FrankaGrasp
Grasp algorithms on Franka robot
From terminal1 type:
```bash
cd FrankaGrasp
source devel/setup.bash
roslaunch gpd_bridge vrep.launch
```
Open another terminal2 type:
```bash
cd FrankaGrasp
source devel/setup.bash
rosrun gpd_bridge pick_and_place_pub
```
Terminal3:
```
cd FrankaGrasp
source devel/setup.bash
roslaunch gpd tutorial2.launch
Terminal4:
```
cd FrankaGrasp
source devel/setup.bash
python src/gpd/scripts/select_grasp.py
```
Terminal5:
```
cd FrankaGrasp
source devel/setup.bash
rosrun pcl_ros pcd_to_pointcloud src/gpd/tutorials/ring12.pcd
```
On the very same terminal5 after press enter in terminal 4 and wait for execution of the algorithm and type
```
 rostopic pub --once /from_to std_msgs/Int16MultiArray "layout:
'''
add after data: in terminal [0,1] for the robot to pick the ring from position 0 to place in position 1, [1,2] to pick in position 1 place in position 2 and so for.

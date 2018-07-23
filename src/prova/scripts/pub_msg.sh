#!/bin/bash
rostopic pub /haf_grasping/input_pcd_rcs_path std_msgs/String "$(rospack find haf_grasping)/data/ring12.pcd" -1
rostopic pub --once /from_to std_msgs/Int16MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0 , 1]"
rostopic pub --once /from_to std_msgs/Int16MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [1 , 2]"
rostopic pub --once /from_to std_msgs/Int16MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [1 , 2]"
rostopic pub --once /from_to std_msgs/Int16MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [2 , 0]"

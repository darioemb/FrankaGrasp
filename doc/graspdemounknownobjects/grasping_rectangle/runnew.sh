#!/bin/sh
#rostopic pub /SS/doSingleShot std_msgs/String "asdf" -1
`rospack find grasping_rectangle`/bin/rank /tmp/scene0001r.png /tmp/background.png /tmp/scene0001.txt `rospack find grasping_rectangle`/model/model.txt /tmp/

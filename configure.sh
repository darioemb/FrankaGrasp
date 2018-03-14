CURR=$PWD

catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=$CURR/lib/libfranka/build

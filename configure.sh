CURR=$PWD

echo -e "\e[33mInstalling panda-moveit-config dependencies\e[39m"
sudo apt-get install ros-kinetic-panda-moveit-config
echo -e "\e[33mInstalled dependencies.\e[39m"

echo -e "\e[33mCOMPILING libfranka...\e[39m"
cd libfranka
if [ -d "lib/libfranka/build" ]; then
	echo -e "\e[33mDirectory build found. Clean your workspace or leave so it is and continue manually!!\e[39m"
else
	cd lib/libfranka
	mkdir build
	cd build

	cmake -DCMAKE_BUILD_TYPE=Release ..
	cmake --build .

	echo -e "\e[32m######### libfranka compiled correctly!\e[39m"
fi

cd $CURR
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=$CURR/lib/libfranka/build

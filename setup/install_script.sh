#!/bin/bash
echo "start installation..."
echo "[navigation] installing..."
echo "y" | sudo apt install ros-kinetic-navigation
echo "[navigation] installed!"
sleep 2s
echo "[robot-pose-publisher] installing..."
echo "y" | sudo apt install ros-kinetic-robot-pose-publisher
echo "[robot-pose-publisher] installed!"
sleep 2s
echo "[gmapping] installing..."
echo "y" | sudo apt install ros-kinetic-gmapping
echo "[gmapping] installed!"
sleep 2s
echo "[teb-local-planner] installing..."
echo "y" | sudo apt install ros-kinetic-teb-local-planner
echo "[teb-local-planner] installed!"
sleep 2s
echo "[openslam-gmapping] installing..."
echo "y" | sudo apt install ros-kinetic-openslam-gmapping
echo "[openslam-gmapping] installed!"
sleep 2s
echo "[hector-nav-msgs] installing..."
echo "y" | sudo apt install ros-kinetic-hector-nav-msgs
echo "[hector-nav-msgs] installed!"
sleep 2s
echo "[urg-node] installing..."
echo "y" | sudo apt install ros-kinetic-urg-node
echo "[urg-node] installed!"
sleep 2s
echo "#################################"
echo "All ROS packages are installed!"
sleep 2s
echo "[fping] installing..."
echo "y" | sudo apt install fping
echo "[fping] installed!"
sleep 2s
echo "[openssh] installing..."
echo "y" | sudo apt install openssh-server openssh-client
echo "[openssh] installed!"
sleep 2s
echo "[network-manager] installing..."
echo "y" | sudo apt install network-manager
echo "[network-manager] installed!"
echo "#################################"
echo "All tools are installed!"
sleep 2s
#if [ -z "$(grep "alias cat_make=" ~/.bashrc)" ]
#then
#    echo "[.bashrc] configuring..."
#    echo "alias cat_make='cd ~/catkin_ws/ && catkin_make && source devel/setup.bash && . ~/catkin_ws/devel/setup.bash'" >> ~/.bashrc
#    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
#    echo "source /home/gtdollar/catkin_ws/devel/setup.bash" >> ~/.bashrc
#    echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/gtdollar/catkin_ws/src" >> ~/.bashrc
#    source ~/.bashrc
#    echo "[.bashrc] configured!"
#else
#    echo "[.bashrc] configured!"
#fi
echo "#################################"
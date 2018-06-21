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
#hokuyo laser
echo "[urg-node] installing..."
echo "y" | sudo apt install ros-kinetic-urg-node
echo "[urg-node] installed!"
sleep 2s
#yocs_velocity_smoother package
echo "[ecl] installing..."
echo "y" | sudo apt install ros-kinetic-ecl
echo "[ecl] installed!"
sleep 2s
#robot_localization package
echo "[geographic-msgs] installing..."
echo "y" | sudo apt install ros-kinetic-geographic-msgs
echo "[geographic-msgs] installed!"
sleep 2s
echo "[tf2-geometry-msgs] installing..."
echo "y" | sudo apt install ros-kinetic-tf2-geometry-msgs
echo "[tf2-geometry-msgs] installed!"
sleep 2s
echo "#################################"
echo "All ROS packages are installed!"
sleep 2s
echo "[fping] installing..."
echo "y" | sudo apt install fping
echo "[fping] installed!"
sleep 2s
echo "[wmctrl] installing..."
echo "y" | sudo apt install wmctrl
echo "[wmctrl] installed!"
sleep 2s
echo "[openssh] installing..."
echo "y" | sudo apt install openssh-server openssh-client
echo "[openssh] installed!"
sleep 2s
echo "[network-manager] installing..."
echo "y" | sudo apt install network-manager
echo "[network-manager] installed!"
sleep 2s
echo "[libusb-dev] installing..."
echo "y" | sudo apt-get install libusb-dev
echo "[libusb-dev] installed!"
sleep 2s
echo "[libspnav-dev] installing..."
echo "y" | sudo apt-get install libspnav-dev
echo "[libspnav-dev] installed!"
echo "#################################"
echo "All tools are installed!"
echo "#################################"
echo "start changing udev rules..."
path=$(cd `dirname $0`; pwd)
sudo sh $path/create_udevrules_script.sh
sleep 2s
echo "#################################"
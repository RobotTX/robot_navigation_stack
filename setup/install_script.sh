#!/bin/bash
echo "#################################"
echo "START ROS PACKAGES INSTALLMENT..........."
echo "#################################"
echo "[navigation] installing..."
echo "y" | sudo apt install ros-kinetic-navigation
echo "[navigation] installed!"
sleep 2s
echo "[robot-pose-publisher] installing..."
echo "y" | sudo apt install ros-kinetic-robot-pose-publisher
echo "[robot-pose-publisher] installed!"
sleep 2s
echo "[openslam-gmapping] installing..."
echo "y" | sudo apt install ros-kinetic-openslam-gmapping
echo "[openslam-gmapping] installed!"
sleep 2s
echo "[hector-nav-msgs] installing..."
echo "y" | sudo apt install ros-kinetic-hector-nav-msgs
echo "[hector-nav-msgs] installed!"
sleep 2s
#teb_local_planner package
echo "[teb-local-planner] installing..."
echo "y" | sudo apt install ros-kinetic-teb-local-planner
echo "[teb-local-planner] installed!"
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
#cv_bridge package
echo "[cv-bridge] installing..."
echo "y" | sudo apt install ros-kinetic-cv-bridge
echo "[cv-bridge] installed!"
sleep 2s
#image_transport package
echo "[image-transport] installing..."
echo "y" | sudo apt install ros-kinetic-image-transport
echo "[image-transport] installed!"
sleep 2s
#usb-cam package
echo "[usb-cam] installing..."
echo "y" | sudo apt install ros-kinetic-usb-cam
echo "[usb-cam] installed!"
sleep 2s
#opencv package
echo "[opencv] installing..."
echo "y" | sudo apt install ros-kinetic-opencv3
echo "[opencv] installed!"
sleep 2s
echo "#################################"
echo "All ROS PACKAGES ARE INSTALLED!"
echo "#################################"
echo ""
echo "#################################"
echo "REMOVE DUPLICATE PACKAGES..........."
echo "#################################"
echo "y" | sudo apt remove ros-kinetic-move-base
echo "[move-base] removed!"
sleep 2s
echo "y" | sudo apt remove ros-kinetic-amcl
echo "[amcl] removed!"
sleep 2s
echo "y" | sudo apt remove ros-kinetic-costmap-converter
echo "[costmap-converter] removed!"
sleep 2s
echo "#################################"
echo "All DUPLICATE PACKAGES ARE REMOVED!"
echo "#################################"
echo ""
echo "#################################"
echo "START LINUX TOOLS INSTALLMENT..........."
echo "#################################"
sleep 2s
echo "[fping] installing..."
echo "y" | sudo apt install fping
echo "[fping] installed!"
sleep 2s
echo "[vim] installing..."
echo "y" | sudo apt install vim
echo "[vim] installed!"
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
sleep 2s
#sudo play voice files
echo "y" | sudo apt install sox
echo "[sox] installed!"
sleep 2s
#support mp3 format
echo "y" | sudo apt install libsox-fmt-all
echo "[libsox-fmt-all] installed!"
echo "#################################"
echo "All LINUX TOOLS ARE INSTALLED!"
echo "#################################"
echo ""
echo "#################################"
echo "GIVE FILES PERMISSION..........."
echo "#################################"
sudo find ~/catkin_ws/src/robot_navigation_stack/ -name "*.cfg" -exec chmod +x {} \;
echo "#################################"
echo "FILES PERMISSION ARE GIVEN!"
echo "#################################"
echo ""
echo "#################################"
echo "MODIFY UDEV RULES..........."
echo "#################################"
echo "start changing udev rules..."
path=$(cd `dirname $0`; pwd)
sudo sh $path/create_udevrules_script.sh
sleep 2s
echo "#################################"
echo "UDEV RULES MODIFIED!"
echo "#################################"
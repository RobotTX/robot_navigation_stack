#!/bin/bash
echo "start installation..."
echo "installing navigation..."
echo "y" | sudo apt install ros-kinetic-navigation
echo "navigation installed!"
sleep 2s
echo "installing robot-pose-publisher..."
echo "y" | sudo apt install ros-kinetic-robot-pose-publisher
echo "robot-pose-publisher installed!"
sleep 2s
echo "installing gmapping..."
echo "y" | sudo apt install ros-kinetic-gmapping
echo "gmapping installed!"
sleep 2s
echo "installing openslam-gmapping..."
echo "y" | sudo apt install ros-kinetic-openslam-gmapping
echo "openslam-gmapping installed!"
sleep 2s
echo "installing hector-nav-msgs..."
echo "y" | sudo apt install ros-kinetic-hector-nav-msgs
echo "hector-nav-msgs installed!"
sleep 2s
echo "installing urg-node..."
echo "y" | sudo apt install ros-kinetic-urg-node
echo "urg-node installed!"
sleep 2s
echo "all packages are installed!"
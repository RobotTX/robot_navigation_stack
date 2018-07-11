#!/bin/bash
echo "y" | sudo apt install festival
echo "[festival] installed!"
sleep 2s
echo "y" | sudo apt install libespeak-dev
echo "[libespeak-dev] installed!"
sleep 2s
echo "y" | sudo apt install libsndfile1-dev
echo "[libsndfile1-dev] installed!"
sleep 2s
echo "y" | sudo apt install libpulse-dev
echo "[libpulse-dev] installed!"
sleep 2s
echo "y" | sudo apt install libncurses5-dev
echo "[libncurses5-dev] installed!"
sleep 2s
echo "y" | sudo apt install libestools-dev
echo "[libestools-dev] installed!"
sleep 2s
echo "y" | sudo apt install festival-dev
echo "[festival-dev] installed!"
sleep 2s
echo "y" | sudo apt install festival-dev
echo "[festival-dev] installed!"
sleep 2s
echo "#################################"
echo "All tts tools are installed!"
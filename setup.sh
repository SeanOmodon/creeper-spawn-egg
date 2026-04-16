#!/bin/bash
set -e
sudo apt update
sudo apt install -y python3-opencv python3-picamera2 python3-rpi.gpio python3-numpy python3-pip
pip3 install --break-system-packages -r requirements.txt
echo "Setup complete."
#!/bin/bash

# check Ubuntu version
source /etc/os-release

if [[ $UBUNTU_CODENAME != 'jammy' ]]
then
    echo "Ubuntu 22.04.1 LTS (Jammy Jellyfish) is required"
    echo "You are using $VERSION"
    exit 1
fi

### Get directory where this script is installed
BASEDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

sudo pip install simple-pid

source /opt/ros/humble/setup.bash
cd ~/ros_ws
colcon build --executor sequential --symlink-install

cd ~/trr/services
sudo ln -s $(realpath .)/button-monitor.service /etc/systemd/system/
sudo ln -s $(realpath .)/stop-line-detector.service /etc/systemd/system/
sudo ln -s $(realpath .)/stop-timer.service /etc/systemd/system/
sudo ln -s $(realpath .)/race-runner.service /etc/systemd/system/
sudo ln -s $(realpath .)/wall-follower.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable button-monitor
sudo systemctl enable stop-line-detector
sudo systemctl enable stop-timer
sudo systemctl enable race-runner
sudo systemctl enable wall-follower

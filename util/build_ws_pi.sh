#!/usr/bin/env bash

cd /home/glados/mnt
git pull origin main

cd "/home/glados/mnt/glados_ws
colcon build --symlink-install
source install/setup.bash
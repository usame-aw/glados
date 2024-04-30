#!/usr/bin/env bash

docker stop glados-pi > /dev/null 2>&1 || true
docker rm glados-pi > /dev/null 2>&1 || true

docker run -d \
    --privileged \
    --net=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/glados:/home/glados/mnt/ \
    -v /dev:/dev \
    -e DISPLAY=${DISPLAY} \
    -u 1000 \
    -w /home/glados/mnt/glados_ws \
    --name glados-pi \
    --restart unless-stopped \
    glados-pi:latest \
    sleep infinity

#extra convenient stuff
docker exec glados-pi bash -c "sudo chmod 777 /dev/ttyUSB0" 
docker exec glados-pi bash -c "colcon build --symlink-install"
docker exec glados-pi bash -c "echo 'source /opt/ros/humble/install/setup.bash' >> /home/glados/.bashrc"
docker exec glados-pi bash -c "echo 'source /home/glados/mnt/glados_ws/install/setup.bash' >> /home/glados/.bashrc"
docker exec glados-pi bash -c "echo 'export PATH=\$PATH:/home/glados/mnt/util/' >> /home/glados/.bashrc"
docker exec glados-pi bash -c "echo 'export ROS_DOMAIN_ID=21' >> /home/glados/.bashrc"

# GLaDOS, a ROS2 & NAV2 based EE Capstone Differential Drive Robot with 3D Mapping and Autonomous Navigation Capabilties 
This repository contains the source code for the GLaDOS project and some docker containers for development, that document/isolate the dependencies and the environment. The development is meant to be carried out using the pc docker container while the deployment happens on the raspberry pi 5 docker container.

## Development Computer Prerequisites

1. [Install Docker Engine using the convience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script) 

2. [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

3. [Configure Docker to start on boot](https://docs.docker.com/engine/install/linux-postinstall/#configure-docker-to-start-on-boot-with-systemd)

## Raspberry Pi 5 Prerequisites
1. Install an Ubuntu OS on the RPI5
2. Follow the same steps in Development Computer Prerequisite

## PC Setup

For maximum convenience:
 
1. Install the _Remote Development_ extension in VSCode.

2. Clone the repo (or forked repo) to a directory named glados on your desktop ```git clone https://github.com/usame-aw/glados.git ~/Desktop/glados```. 
    
    The directory structure should look as follows

    ```
    Desktop
    ├── glados
        ├── .devcontainer
        ├── glados_ws
        └── util
    ```

3. Allow docker to launch GUIs by running ```xhost +local:docker```. This can optionally be _added to the .bashrc_ for extra convience. Ignore any warnings resulting from this command.

4. Open VSCode's command pallete (CTRL + SHIFT + P) and run ```Dev Containers: Rebuild and Reopen in Container```

5. Wait for the building process to finish. The building process might take upwards of 10-15 mins depending on your internet connection.

6. Check your installation by running any ros2 command.

7. After building once, run ```Dev Containers: Reopen in Container``` in the command pallete to resume working on the labs.

8. Create a branch for the feature you want to develop. Test the feature very carefully on that branch then merge and push to main.

## RPI5 Setup

- Clone this repo to ~/glados. The directory structure should look similar to that on the PC side.
  
- To build the docker container (create an image) navigate to util directory and run the script ```./build_image_pi.sh```

- To run the docker container (instantiate and image) run the ```./create_glados_pi.sh``` script.

- After pulling the latest changes from main, run ```colcon build --symlink-install``` from the glados_ws directory.
  
## Basic Docker CLI

## Using the Robot


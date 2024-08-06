# CMU VLA Challenge Docker Instructions

Prior to following these instructions, make sure you have pulled this repo and copied it to your `/home/$USER` folder. If you want to use a different path, refer to the note under the section [Run and Modify Docker Image](#run-and-modify-docker-image).

## Install Docker

### 1) For computers without a Nvidia GPU

Install Docker and grant user permission:
```
curl https://get.docker.com | sh && sudo systemctl --now enable docker
sudo usermod -aG docker ${USER}
```
Make sure to **restart the computer**, then install additional packages:
```
sudo apt update && sudo apt install mesa-utils libgl1-mesa-glx libgl1-mesa-dri
```

### 2) For computers with Nvidia GPUs

Install Docker and grant user permission.
```
curl https://get.docker.com | sh && sudo systemctl --now enable docker
sudo usermod -aG docker ${USER}
```
Make sure to **restart the computer**, then install Nvidia Container Toolkit (Nvidia GPU Driver
should be installed already).

```
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor \
  -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```
```
sudo apt update && sudo apt install nvidia-container-toolkit
```
Configure Docker runtime and restart Docker daemon.
```
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```
Test if the installation is successful, you should see something like below.
```
docker run --gpus all --rm nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi
```
```
Sat Dec 16 17:27:17 2023       
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 525.125.06   Driver Version: 525.125.06   CUDA Version: 12.0     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
| 24%   50C    P0    40W / 200W |    918MiB /  8192MiB |      3%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+
                                                                               
+-----------------------------------------------------------------------------+
| Processes:                                                                  |
|  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
|        ID   ID                                                   Usage      |
|=============================================================================|
+-----------------------------------------------------------------------------+
```

## Run and Modify Docker Image
Move the entire `CMU-VLA-Challenge` repo to your local `/home/$USER` folder so that it is the working directory in the docker image.

Note: If you choose to place it elsewhere, you'll have to modify the `working_dir` parameter in the docker compose file(s) to match the path to your cloned repo.

Inside the `CMU-VLA-Challenge` folder, allow remote X connection:
```
xhost +
```
Go inside this folder in terminal.
```
cd CMU-VLA-Challenge/docker/
```

For computers **without a Nvidia GPU**, compose the Docker image and start the container:
```
docker compose -f compose.yml up --build -d
```
For computers **with Nvidia GPUs**, use the `compose_gpu.yml` file instead (creating the same Docker image, but starting the container with GPU access):
```
docker compose -f compose_gpu.yml up --build -d
```
Access the running container:
```
docker exec -it ubuntu20_ros bash
```

## Launch Entire System
Follow the instructions in the "Set Up" section of [system/unity](../system/unity) to set up the simulator with Unity environment models. To test whether the simulator was correctly set up, go inside the folder and run:
```
./system_bring_up.sh
```
Go inside the `ai_module` folder and compile and set up the package:
```
catkin_make
```
The system can then be launched altogether with the script under the root repository directory: 
```
./launch.sh
```
You should see both the simulator launching in RViz and a terminal prompt asking for text input.

The prompt will ask you to type in a question or command and the system will move accordingly. As the system is running with a "dummy model" by default, it simply parses the type of statement and returns the appropriate response type with arbitrary values. The behavior of the dummy model for different language inputs is as follows: 
- "how many...": prints out a number in terminal
- "find the...": highlights the object with a visualization marker and navigates to it
- anything else: sends a series of fixed waypoints

If you use the control panel to navigate the vehicle, to resume waypoint navigation afterwards, click the 'Resume Navigation to Goal' button. The contents under the [ai_module](../ai_module) folder can be modified and the [dummy_vlm](../ai_module/src/dummy_vlm/) package replaced with yours.


## Push Docker Image
After you are done with the modifications, push the image to [Docker Hub](https://hub.docker.com/). To do this, create a Docker Hub account and login to the account from another terminal (not the terminal accessing the container):
```
docker login -u [DOCKERHUB_USERNAME]
```
Commit the container to a Docker image and push the image to Docker Hub with the commands below. To view [IMAGE_ID], use the ``docker images`` command and pick the Docker image that was last created:
```
docker commit ubuntu20_ros ubuntu20_ros
docker tag [IMAGE_ID] [DOCKERHUB_USERNAME]/ubuntu20_ros:cmu_vla_challenge_simulation
docker push [DOCKERHUB_USERNAME]/ubuntu20_ros:cmu_vla_challenge_simulation
```

## Pull Docker Image and Check

Follow these steps to pull the image and verify the contents, as the image will be pulled in the same way for the challenge evaluation. 

Allow remote X connection:
```
xhost +
```
Pull the image. To do this, you need to first remove the uploaded Docker image from your computer:
```
docker pull [DOCKERHUB_USERNAME]/ubuntu20_ros:cmu_vla_challenge_simulation
```
For computers **without a Nvidia GPU**, start the container. Replace [IMAGE_ID] in the command line. You can view [IMAGE_ID] with the ``docker images`` command:
```
docker run -it --rm --privileged -e DISPLAY -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=/tmp/.docker.xauth -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /etc/localtime:/etc/localtime:ro \
  -v /dev/input:/dev/input -v /dev/bus/usb:/dev/bus/usb:rw -v /home/$USER:/home/$USER:rw \
  --network=host [IMAGE_ID]
```
For computers **with Nvidia GPUs**, start the container with `--gpus all` flags:
```
docker run --gpus all -it --rm --privileged -e DISPLAY -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=/tmp/.docker.xauth -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /etc/localtime:/etc/localtime:ro \
  -v /dev/input:/dev/input -v /dev/bus/usb:/dev/bus/usb:rw -v /home/$USER:/home/$USER:rw \
  --network=host [IMAGE_ID]
```
Launch the overall system:
```
launch.sh
```

## Use Different Base Image

To use a different base image, e.g. with CUDA pre-installation, you may use the provided `Dockerfile_base` file. First, rename `Dockerfile` to something else and rename 'Dockerfile_base' to `Dockerfile`. 

Edit the base image name on the first line of the file and point it to the image you would like to use. Then, follow instructions in the [Run and Modify Docker Image](#run-and-modify-docker-image) section above to start the container and access it. 

Pull this `CMU-VLA-Challenge` repo and place it in your local `/home/$USER` folder as before.

In a terminal, go to the [unity](../system/unity/) and [ai_module](../ai_module) folders to recompile the repositories by removing the `build` and `devel` folders followed by the `catkin_make` command. Note that the containers are started with access to your local `/home/$USER` folder for easy copy/paste to and from the images.

## Other Useful Docker Commands

Check running containers:
```
docker ps -a
```
Stop and remove all running containers:
```
docker stop $(docker ps -a -q)
docker rm $(docker ps -a -q)
```
Check Docker images:
```
docker images
```
Remove a Docker image (after stopping and removing the running container):
```
docker rmi [REPO_NAME]:[TAG]
```
Remove dangling Docker images:
```
docker image prune
```

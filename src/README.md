## Usage

Instructions for running the nodes using a local installation of ROS. This is mainly intended for development purposes, while the Docker containers should be used in production.

### Setup

[Setup ROS catkin workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace), `git clone` this repository to `catkin_ws/src/` and build the catkin workspace by running 'catkin_make' in the catkin_ws directory.
```
cd ~/catkin_ws/src/
git clone git@github.com:Konenako/Ohtuprojekti-kesa2020.git
cd ..
catkin_make
```

If you haven't already, install the dependencies by running poetry in the repository's root directory.
```
cd src/Ohtuprojekti-kesa2020/
poetry install
```

Enter the virtual environment.
```
poetry shell
```

Source the catkin workspace (must be done after entering the virtual environment).
```
source ../../devel/setup.bash
```

### Running nodes

All the instructions in this section presume you are in the poetry shell, have sourced the setup.bash file and are in the src  directory (catkin_ws/src/Ohtuprojekti-kesa2020/src/).


For the ROS nodes to communicate, roscore must be running on the system.

```
roscore
```

All the other nodes can be started using rosrun to run the respective python file.
```
rosrun konenako node_xxx.py
```

Currently available nodes, their source files and functions:

|Node    | File     | Function  |
| ------ | -------- | --------- |
|roscamera|node_camera.py|Publish a video feed to a topic|
|rosdetector|node_detector.py|Run a TF model on a video feed|
|rosqrnode|node_qr_reader.py|Run QR detection on a video feed|
|rosprinter|node_printer.py|Display the result feed of all nodes|
|rosinput|node_input.py|Send commands to nodes|


## Docker

Building the image

`sudo docker build --tag konenako:1.0 .`

Create network for communicating with roscore

`sudo docker network create rosnet`

Start up roscore
```
sudo docker run -it --rm \
--net rosnet \
--name master \
ros:melodic-ros-core \
roscore
```
Start up rostest node in another console
```
sudo docker run -it --rm \
    --net rosnet \
    --name master \
    --env ROS_HOSTNAME=rostest \
    --env ROS_MASTER_URI=http://master:11311 \
    rostest:1.0
```
Attaching camera devices to the container can be done by adding parameters to node container startup

`--device` argument, for example `--device /dev/video0`

### Docker compose

Alternatively the above can be done with docker compose, with the following on the root directory of the project.

`docker-compose build`

`docker-compose up`

For sending messages to rostest.py attach to input container with another terminal:

`sudo docker attach rosinput`

For seeing recieved messages in separate terminal use:

`sudo docker attach rostest`

For seeing recieved information about the objects use:

`sudo docker attach rosprinter`


Attaching devices like webcams
https://docs.docker.com/compose/compose-file/#devices

## Versions

Tested to work on Docker version 19.03.8-ce, Docker-compose version 1.25.5 and poetry 1.0.5.

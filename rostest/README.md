## Usage

Setup ROS catkin workspace, `git clone` to `catkin_ws/src/` and build the catkin workspace.


`pipenv shell`
```
source ../../devel/setup.bash
cd rostest
rosrun rostest main.py
```

## Docker

Building the image

`sudo docker build --tag rostest:1.0 .`

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
    --name rostest \
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

`sudo docker attach rostest`


Attaching devices like webcams
https://docs.docker.com/compose/compose-file/#devices

## Versions

Tested to work on Docker version 19.03.8-ce and Docker-compose version 1.25.5

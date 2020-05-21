Setup catkin workspace, `git clone` to `catkin_ws/src/`

## Usage

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
Start up rostest node
```
sudo docker run -it --rm \
    --net rosnet \
    --name rostest \
    --env ROS_HOSTNAME=rostest \
    --env ROS_MASTER_URI=http://master:11311 \
    rostest:1.0 \
    bash
```
Run commands on rostest node bash

`cd /catkin_ws/src/ohtu && pipenv shell`

`source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd rostest && rosrun rostest main.py`

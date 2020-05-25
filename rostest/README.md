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

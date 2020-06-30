# Konenako
Ohjelmistotuotantoprojekti kesä 2020: Robotin konenäkö mikropalveluna  
[![CircleCI](https://circleci.com/gh/Konenako/Ohtuprojekti-kesa2020.svg?style=svg)](https://circleci.com/gh/Konenako/Ohtuprojekti-kesa2020) [![codecov](https://codecov.io/gh/Konenako/Ohtuprojekti-kesa2020/branch/master/graph/badge.svg)](https://codecov.io/gh/Konenako/Ohtuprojekti-kesa2020) [![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)  
This repository contains the work for the University of Helsinki course called *Ohjelmistotuotantoprojekti, kesä 2020*.  

The goal is to implement computer vision as microservices utilizing the [Robot Operating System (ROS)](https://www.ros.org/) framework.  

Currently we provide one ROS package named 'konenako', which is implemented in the [src](https://github.com/Konenako/Ohtuprojekti-kesa2020/tree/master/src)-folder. This package provides several [ROS nodes](#nodes) that can be run individually.

We also provide already configured Dockerfiles for both x86-64 and armv7 architectures, so you can easily build every node into its own docker-image. For Arm architectures we also provide precompiled docker images through [Docker Hub](https://hub.docker.com/r/ohtukonenako/ohtuprojekti_kesa2020), which are tested to work on a Raspberry PI 3B+ based [TurtleBot3](http://www.robotis.us/turtlebot-3/) robot.


## Documentation

[Sprint Backlog](https://github.com/Konenako/Ohtuprojekti-kesa2020/projects)

[Product backlog](https://docs.google.com/spreadsheets/d/1jyyo4Vl1vxXgr6DDcG-a-Rb9Xx-2B-TYjd2KzZyrj3M/edit?usp=sharing)

[ROS nodes API](https://github.com/Konenako/Ohtuprojekti-kesa2020/blob/master/documentation/Ros_node_APIs.md)

[ROS nodes structure](https://github.com/Konenako/Ohtuprojekti-kesa2020/blob/master/documentation/program_structure.png?raw=true)

[Code documentation](https://konenako.github.io/Ohtuprojekti-kesa2020/)

### Definition of Done
* All code should be formatted with yapf to follow [PEP8](https://www.python.org/dev/peps/pep-0008/#introduction)
* All code should be clearly commented
   * Every module and class should have an "docstring", formatted in [Doxygen style](https://www.doxygen.nl/manual/docblocks.html#pythonblocks), [Hints and tips for doxygen in Finnish](https://docs.google.com/document/d/1dO_enSIPJnerTgj0mP3ikSAUP4uKJe4mQp6P-w6SBbI/edit#heading=h.3dfaehlwii74)
   * All non trivial code should be commented with the same style
   * [Type hinting](https://docs.python.org/3.7/library/typing.html) should be used
* All CircleCI tests must pass before merging to master
* Pull requests need at least 2 approvals before merging to master
* Documentation should be updated whenever any functionality is changed or added

### Repository structure
```bash
.
├── documentation
├── resources
│   ├── images                      - Contains all the images, for tests etc.
│   ├── python                      - Contains various helperscripts
│   ├── tflite_models               - Model and labelfiles
│   └── videos                      - Contains all the videos, for tests etc.
├── src                             - ROS-package directory
│   ├── msg                         - ROS messages specifications
│   ├── scripts                     - Nodes are placed here as executable python-files
│   │   ├── detector                - Python packages can be placed here
│   │   ├── config                  - Package for various configs
│   │   │   └── constants.py        - Constants are defined in this module
│   │   ├── ...
│   │   ├── <package_n>
│   │   ├── node_camera.py          - node executables are prefixed with the name "node_"
│   │   ├── node_<..>.py
│   │   └── node_qr_reader.py
│   ├── srv                         - ROS services specifications 
│   └── tests                       - Python unittests
```

## Running the application

Running program/nodes locally requires [installing ROS](http://wiki.ros.org/ROS/Installation).
Running program/nodes with docker requires [installing Docker](https://docs.docker.com/engine/install/).

### Roscore & rosnet
All of the other instructions require ros master to be running. You can use docker network and start up the master with the following commands, or use an already running instance of master by modifying the `ROS_MASTER_URI` environment variable to the correct address for the running core.
```console
sudo docker network create rosnet
```
```console
sudo docker run -it --rm \
--net rosnet \
--name master \
ros:melodic-ros-core \
roscore
```


### Building for Raspberry Pi
The Docker image of master branch is automatically built for arm32v7 architecture with qemu-emulator, and deployed to the docker hub.
If you want to build the image on your own x86_64 machine, you can use the same commands that are in the `.travis.yml`-file.

To retrieve the built image from docker hub:
``sudo docker pull ohtukonenako/ohtuprojekti_kesa2020:latest``

### Running on Raspberry Pi

Confirm that roscore and rosnet are set up [instructions](#roscore--rosnet)

Start up the image, change ROS_MASTER_URI environment variable to correct address for the ros master, if you did not start up the master with the command previously given. 
```
sudo docker run -it --rm \
    --net rosnet \
    --privileged \
    --name konenako \
    --env ROS_HOSTNAME=konenako \
    --env ROS_MASTER_URI=http://master:11311 \
    ohtukonenako/ohtuprojekti_kesa2020:latest bash -c "cd src/ohtu && poetry run /bin/bash -c 'source ../../devel/setup.bash && ROS_HOME=/catkin_ws/src/ohtu roslaunch test.launch'"
```
### Mounting models
You can mount directories to the container with `-v source:dest` flag, for example, `-v /home/konenako/models:/models` and then give model_path as `/models/model1.tflite`, to use model located in `/home/konenako/models/model1.tflite`.
In a similar manner, your own custom .launch files can be mounted to the container, and then the container started with your launch file by modifying the path given to the roslaunch.

### Using devices
When the container is started with --privileged flag, you can use devices of the host normally. `/dev/video0` usually corresponds to the first camera of the host.

### Configuring & debugging
You must connect to the ros core, for example by running the image with only bash opened.
```
sudo docker run -it --rm \
    --net rosnet \
    --name debug \
    --privileged \
    --env ROS_HOSTNAME=debug \
    --env ROS_MASTER_URI=http://master:11311 \
    -t ohtukonenako/ohtuprojekti_kesa2020:latest bash
```
Now you can debug as you usually would with rostopic etc.

To configure, you can use ros parameters or make your own modified version of the .launch file. To use your own mount file follow instructions in [mounting models](#mounting-models).

Instructions should work at least on Ubuntu.

### Running with docker (for x86_64)
Confirm that roscore and rosnet are set up [instructions](#roscore--rosnet)
```console
sudo docker run -it --rm \
    --net rosnet \
    --name asd \
    --privileged \
    --env ROS_HOSTNAME=asd \
    --env ROS_MASTER_URI=http://master:11311 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -t konenako bash -c "cd src/ohtu && poetry run /bin/bash -c 'source ../../devel/setup.bash && QT_X11_NO_MITSHM=1 ROS_HOME=/catkin_ws/src/ohtu roslaunch test.launch'"
```

### Running locally (for x86_64)

Inside repository's root directory:

```console
source /opt/ros/$(rosversion -d)/setup.bash
catkin_make
poetry install --no-dev
poetry shell
source devel/setup.bash
ROS_HOME=`pwd` roslaunch test.launch
```

### Running nodes individually (for x86_64)

**All the instructions in this section presume you are in the repository's root directory.**

For the ROS nodes to communicate, roscore must be running on the system. So make sure it is running
```console
roscore
```

In another terminal setup catkin workspace inside project folder
```console
cd /path/to/project/folder
source /opt/ros/$(rosversion -d)/setup.bash
catkin_make
```

and set needed parameters
Note: if you have the environment variable ROS_NAMESPACE set, you must  the /konenako/ -prefix from the commands.

```console
rosparam set konenako/camhz 30
rosparam set konenako/combine_results True
rosparam set konenako/video_source <resources/videos/test.mp4 | /dev/video0>
```

node_detector_control needs parameters for every detector to be run

```console
rosparam set konenako/init_detectors/object_detect/detect_on True
rosparam set konenako/init_detectors/object_detect/frequency 42
rosparam set konenako/init_detectors/object_detect/label_path resources/tflite_models/mscoco_complete_labels
rosparam set konenako/init_detectors/object_detect/model_path resources/tflite_models/ssd_mobilenet_v1_1_metadata_1.tflite
rosparam set konenako/init_detectors/object_detect/score_threshold 0.3

rosparam set konenako/init_detectors/QR/detect_on True
rosparam set konenako/init_detectors/QR/frequency 5
```

<a name="nodes"></a>
Currently available nodes, their source files and functions:

|Node    | File     | Function  | Needed parameters |
| ------ | -------- | --------- | ----------------- |
|camera|node_camera.py|Publish a video feed to a topic| konenako/camhz konenako/video_source |
|detector_control_node|node_detector_control.py|Run a TF model and QR detector on a video feed| konenako/combine_results konenako/init_detectors/object_detect/detect_on konenako/init_detectors/object_detect/frequency konenako/init_detectors/object_detect/label_path konenako/init_detectors/object_detect/model_path konenako/init_detectors/object_detect/score_threshold konenako/init_detectors/QR/detect_on konenako/init_detectors/QR/frequency|
|printer|node_printer.py|Display the result feed of all nodes|  |
|drawer|node_drawer.py|Draws observations onto images and shows them with CV2| |

**For each node do the following:**

Open up new terminal and go to project folder.

Create and activate virtual environment and source catkin files
```console
cd /path/to/project/folder
poetry install
poetry shell
source devel/setup.bash
```

Start any node listed above by running the respective python file with rosrun.
```
ROS_NAMESPACE=konenako rosrun konenako node_xxx.py
```

## Using ROS-services with terminal

Source needed files:
```source dir/to/catkin_workspace/devel/setup.bash```

Run `rosservice list` to get list of available services. Example:
```console
user@computer:~$ rosservice list
/konenako/camera/get_loggers
/konenako/camera/set_logger_level
/konenako/detector_control_node/add_object_detector
/konenako/detector_control_node/combine_toggle
```

Run `rosservice args /some/ROS/service` to get arguments for service. Example:
```console
user@computer:~$ rosservice args /konenako/detector_control_node/combine_toggle
state
```

Run `rosservice call /some/ROS/service [arguments...]` to call service with some arguments. Example:

```console
user@computer:~$ rosservice call /konenako/detector_control_node/combine_toggle true
response: "Combining results set to True"
```

## Commands for virtual environment
[poetry](https://github.com/python-poetry/poetry) is used for managing dependencies

#### Creating virtual environment for development
`poetry install`

#### Activate virtual environment
`poetry shell`
or `poetry run <command>` to run a single command in the environment

#### Additional dependencies
Some libraries must be separately installed, current list:
 * zbar/libzbar0 

### Testing
All python unittests are run from inside the [src](https://github.com/Konenako/Ohtuprojekti-kesa2020/tree/master/src) folder.  
`poetry run python -m unittest`

## Versions

Tested to work on Docker version 19.03.8-ce, Docker-compose version 1.25.5 and poetry 1.0.5.


## Formatting

Run [yapf](https://github.com/google/yapf/) before commits `poetry run yapf -ri .`  
Use `poetry run yapf -rd .` to print diff of changes if needed.

[pep8 style guide](https://www.python.org/dev/peps/pep-0008/)

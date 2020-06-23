# Konenako
Ohjelmistotuotantoprojekti kesä 2020: Robotin konenäkö mikropalveluna  
[![CircleCI](https://circleci.com/gh/Konenako/Ohtuprojekti-kesa2020.svg?style=svg)](https://circleci.com/gh/Konenako/Ohtuprojekti-kesa2020) [![codecov](https://codecov.io/gh/Konenako/Ohtuprojekti-kesa2020/branch/master/graph/badge.svg)](https://codecov.io/gh/Konenako/Ohtuprojekti-kesa2020) [![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)  
This repository contains the work for the University of Helsinki course called *Ohjelmistotuotantoprojekti, kesä 2020*.  

The goal is to implement computer vision as microservices utilizing the [Robot Operating System (ROS)](https://www.ros.org/) framework.  

Currently we provide one ROS package named 'konenako', which is implemented in the [src](https://github.com/Konenako/Ohtuprojekti-kesa2020/tree/master/src)-folder. This package provides several [ROS nodes](#nodes) that can be run individually.

We also provide already configured Dockerfiles for both x86-64 and armv7 architectures, so you can easily build every node into its own docker-image. For Arm architectures we also provide precompiled docker images through [Docker Hub](https://hub.docker.com/r/ohtukonenako/ohtuprojekti_kesa2020), which are tested to work on a Raspberry PI 3B+ based [TurtleBot3](http://www.robotis.us/turtlebot-3/) robot.


## Documentation

[Backlog](https://github.com/Konenako/Ohtuprojekti-kesa2020/projects)

[ROS nodes API](https://docs.google.com/document/d/1ZX2D-lR2-JPhgsHoDeP2K5ocj3jB0FU43SoIwdkePvU)

[ROS nodes structure](https://docs.google.com/drawings/d/1a4bOr0Cu2g_0QJ_u3QxHUjvKshzWlyNOfwyI5jS2Bu8)

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

### Running with docker

`sudo docker network create rosnet`

`sudo docker build -t konenako .`
```console
sudo docker run -it --rm \
--net rosnet \
--name master \
ros:melodic-ros-core \
roscore
```
```console
sudo docker run -it --rm \
    --net rosnet \
    --name asd \
    --env ROS_HOSTNAME=asd \
    --env ROS_MASTER_URI=http://master:11311 \
    -t konenako bash -c "cd src/ohtu && poetry run /bin/bash -c 'source ../../devel/setup.bash && ROS_HOME=/catkin_ws/src/ohtu roslaunch test.launch'"
```
or with script `./docker_runner.sh`

### Running locally 

Inside repository's root directory:

```console
poetry install --no-dev
poetry shell
source /opt/ros/$(rosversion -d)/setup.bash
catkin_make
source devel/setup.bash
ROS_HOME=`pwd` roslaunch test.launch
```
or with script
`./check_run.sh`

### Running nodes individually

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

Set needed parameters

```console
rosparam set konenako/camhz 30
rosparam set konenako/combine_results True
rosparam set konenako/testi/object_detect/detect_on True
rosparam set konenako/testi/object_detect/frequency 42
rosparam set konenako/testi/object_detect/label_path resources/tflite_models/mscoco_complete_labels
rosparam set konenako/testi/object_detect/model_path resources/tflite_models/ssd_mobilenet_v1_1_metadata_1.tflite
rosparam set konenako/testi/object_detect/score_threshold 0.3
rosparam set konenako/video_source resources/videos/test.mp4
```

<a name="nodes"></a>
Currently available nodes, their source files and functions:

|Node    | File     | Function  | Needed parameters |
| ------ | -------- | --------- | ----------------- |
|camera|node_camera.py|Publish a video feed to a topic| `TODO` |
|detector_control_node|node_detector_control.py|Run a TF model and QR detector on a video feed| `TODO` |
|printer|node_printer.py|Display the result feed of all nodes| `TODO` |

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
rosrun konenako node_xxx.py
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

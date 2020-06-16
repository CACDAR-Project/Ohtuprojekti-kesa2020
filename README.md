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

### Definition of Done
* All code should be formatted with yapf to follow [PEP8](https://www.python.org/dev/peps/pep-0008/#introduction)
* All code should be clearly commented
   * Every module and class should have an "docstring", formatted in [Doxygen style](https://www.doxygen.nl/manual/docblocks.html#pythonblocks), [Hints and tips for doxygen in Finnish](https://docs.google.com/document/d/1dO_enSIPJnerTgj0mP3ikSAUP4uKJe4mQp6P-w6SBbI/edit#heading=h.3dfaehlwii74)
   * All non trivial code should be commented with the same style
   * [Type hinting](https://docs.python.org/3.7/library/typing.html) should be used
* All CircleCI tests must pass before merging to master
* Pull requests need at least 2 approvals before merging to master
* Documentation should be updated whenever any functionality is changed or added

### Generating documentation for classes, files etc.    

You can run auto-documentation on root with    
`doxygen`    

The documentation can be found in the documentation-folder and viewed on your browser at ./documentation/html/index.html

### Repository structure
```bash
.
├── documentation
├── src                             - ROS-package directory
│   ├── models                      - Models should be moved here (SUBJECT TO CHANGE)
│   ├── msg                         - ROS messages specifications
│   ├── resources                   - Different resources used in the ROS-package or nodes (SUBJECT TO CHANGE)
│   ├── scripts                     - Nodes are placed here as executable python-files
│   │   ├── detector                - Python packages can be placed here
│   │   ├── ...
│   │   ├── <package_n>
│   │   ├── node_camera.py          - node executables are prefixed with the name "node_"
│   │   ├── node_<..>.py
│   │   └── node_qr_reader.py
│   ├── srv                         - ROS services specifications 
│   └── tests                       - Python unittests
```

## Running the application

### Running with roslaunch
```sudo docker network create rosnet```
```sudo docker build -t konenako .```
```
sudo docker run -it --rm \
--net rosnet \
--name master \
ros:melodic-ros-core \
roscore
```
```
sudo docker run -it --rm \
    --net rosnet \
    --name asd \
    --env ROS_HOSTNAME=asd \
    --env ROS_MASTER_URI=http://master:11311 \
    -t konenako bash -c "cd src/ohtu && poetry run /bin/bash -c 'source ../../devel/setup.bash && ROS_HOME=/catkin_ws/src/ohtu/src roslaunch test.launch'"
```



### Running nodes in Docker with script file

`./docker_runner.sh`

Kills containers that are open, builds and runs containers and attaches new terminals to them.

### Running nodes locally with script file

`./runner.sh`

Deletes `catkin_ws`-folder creates catkin workspace and opens nodes in new gnome terminals.

### Running individual nodes locally

#### Setup

[Setup ROS catkin workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace), `git clone` this repository to `catkin_ws/src/` and build the catkin workspace by running 'catkin_make'.
```
cd ~/catkin_ws/src/
git clone git@github.com:Konenako/Ohtuprojekti-kesa2020.git
cd Ohtuprojekti-kesa2020
catkin_make -C ../../
```

If you haven't already, install the dependencies with poetry.
```
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

#### Running nodes

All the instructions in this section presume you are in the poetry shell, have sourced the setup.bash file and are in the repository's src directory (catkin_ws/src/Ohtuprojekti-kesa2020/src/).


For the ROS nodes to communicate, roscore must be running on the system.

```
roscore
```

All the other nodes can be started using rosrun to run the respective python file.
```
rosrun konenako node_xxx.py
```

<a name="nodes"></a>
Currently available nodes, their source files and functions:

|Node    | File     | Function  |
| ------ | -------- | --------- |
|camera|node_camera.py|Publish a video feed to a topic|
|object_detector|node_object_detector.py|Run a TF model on a video feed|
|qr_detector|node_qr_detector.py|Run QR detection on a video feed|
|rosprinter|node_printer.py|Display the result feed of all nodes|
|rosinput|node_input.py|Send commands to nodes|


### Commands for virtual environment
[poetry](https://github.com/python-poetry/poetry) is used for managing dependencies

#### Creating virtual environment for development
`poetry install`

#### Activate virtual environment
`poetry shell`
or `poetry run <command>` to run a single command in the environment

#### Additional dependencies
Some libraries must be separately installed, current list:
 * zbar/libzbar0 


## Versions

Tested to work on Docker version 19.03.8-ce, Docker-compose version 1.25.5 and poetry 1.0.5.


## Formatting

Run [yapf](https://github.com/google/yapf/) before commits `poetry run yapf -ri .`  
Use `poetry run yapf -rd .` to print diff of changes if needed.

[pep8 style guide](https://www.python.org/dev/peps/pep-0008/)

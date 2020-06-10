[![CircleCI](https://circleci.com/gh/Konenako/Ohtuprojekti-kesa2020.svg?style=svg)](https://circleci.com/gh/Konenako/Ohtuprojekti-kesa2020) [![codecov](https://codecov.io/gh/Konenako/Ohtuprojekti-kesa2020/branch/master/graph/badge.svg)](https://codecov.io/gh/Konenako/Ohtuprojekti-kesa2020) [![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)

## Documentation

[Backlog](https://github.com/Konenako/Ohtuprojekti-kesa2020/projects)

### Documentation for classes, files etc.    

You can run auto-documentation on root with    
`doxygen`    

The documentation can be found in the documentation-folder and viewed on your browser at ../documentation/html/index.html

## Running nodes in Docker with script file

`./docker_runner.sh`

Kills containers that are open, builds and runs containers and attaches new terminals to them.

## Running nodes locally with script file

`./runner.sh`

Deletes `catkin_ws`-folder creates catkin workspace and opens nodes in new gnome terminals.

## Running individual nodes locally

### Setup

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

### Running nodes

All the instructions in this section presume you are in the poetry shell, have sourced the setup.bash file and are in the repository's src directory (catkin_ws/src/Ohtuprojekti-kesa2020/src/).


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


## Versions

Tested to work on Docker version 19.03.8-ce, Docker-compose version 1.25.5 and poetry 1.0.5.


## Formatting

Run [yapf](https://github.com/google/yapf/) before commits `poetry run yapf -ri .`  
Use `poetry run yapf -rd .` to print diff of changes if needed.

[pep8 style guide](https://www.python.org/dev/peps/pep-0008/)

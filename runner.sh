#!/bin/sh

bold=$(tput bold)
normal=$(tput sgr0)

if [ "$BASH_VERSION" = "" ];
then
  echo 'Run with '${bold}'bash runner.sh'${normal}
  exit
fi

if [[ "$VIRTUAL_ENV" = "" ]]
then
  echo 'We are not in venv! Remember to '${bold}'poetry install'${normal}' and '${bold}'poetry shell'${normal}'.'
  exit
fi

ros_distro=$(rosversion -d)
src_folder=rostest
pwd=$(pwd)

echo 'Using '${ros_distro}' distribution'
echo 'Working directory is '${pwd}''
echo 'Assuming that there is a folder '${src_folder}' that contains needed files.'

if [ ! -d ${pwd}"/${src_folder}" ]
then
    echo 'Folder '${bold}${pwd}"/"${src_folder}${normal}' does not exist!'
    exit
fi

if [ -d ${pwd}"/catkin_ws" ]
then
    echo 'Folder '${bold}${pwd}'/catkin_ws'${normal}' deleted!'
    rm -r catkin_ws
fi

echo 'Creating folder '${pwd}'/catkin_ws/src'
mkdir -p ${pwd}/catkin_ws/src

echo 'Copying all files from '${pwd}'/'${src_folder}' to '${pwd}'/catkin_ws/src'
cp -r ${pwd}/${src_folder}/* ${pwd}/catkin_ws/src

echo 'Sourcing /opt/ros/'${ros_distro}'/setup.bash'
source /opt/ros/${ros_distro}/setup.bash

echo 'Removing tensorflow 1.14.0 because it requires package enum34 which prevents catkin_make'
poetry remove tensorflow > /dev/null 2>&1

echo 'Creating catkin workspace with catkin_make to '${pwd}'/catkin_ws'
catkin_make -C ${pwd}/catkin_ws > /dev/null 2>&1

echo 'Adding tensorflow 1.14.0 because it is needed'
poetry add tensorflow==1.14.0 > /dev/null 2>&1

echo 'Sourcing '${pwd}'/devel/setup.bash'
source ${pwd}/devel/setup.bash

roscore &
gnome-terminal --geometry 60x16+0+0 --title="OBJECTS DETECTION" -- /bin/bash -c 'source '${pwd}'/catkin_ws/devel/setup.bash; cd '${pwd}'/catkin_ws/src; rosrun konenako node_detector.py; exec bash' &
gnome-terminal --geometry 60x16+0+359 --title="PRINTER" -- /bin/bash -c 'source '${pwd}'/catkin_ws/devel/setup.bash; cd '${pwd}'/catkin_ws/src; rosrun konenako node_printer.py; exec bash' &
gnome-terminal --geometry 60x16+625+0 --title="INPUT" -- /bin/bash -c 'source '${pwd}'/catkin_ws/devel/setup.bash; cd '${pwd}'/catkin_ws/src; rosrun konenako node_input.py; exec bash' &
gnome-terminal --geometry 60x16+625+359 --title="CAMERA" -- /bin/bash -c 'source '${pwd}'/catkin_ws/devel/setup.bash; cd '${pwd}'/catkin_ws/src; rosrun konenako node_camera.py; exec bash'

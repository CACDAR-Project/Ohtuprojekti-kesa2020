#!/bin/bash

bold=$(tput bold)
normal=$(tput sgr0)
ros_distro=$(rosversion -d)
src_folder=src
pwd=$(pwd)

if [[ "$VIRTUAL_ENV" != "" ]]
then
  echo 'You are in venv! Quit venv with '${bold}'exit'${normal}' or '${bold}'Ctrl+D'${normal}'.'
  exit
fi

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

echo 'Creating catkin workspace with catkin_make to '${pwd}'/catkin_ws'
catkin_make -C ${pwd}/catkin_ws > /dev/null 2>&1

echo 'Sourcing '${pwd}'/catkin_ws/devel/setup.bash'
source ${pwd}/catkin_ws/devel/setup.bash

echo 'Installing poetry dependencies'
poetry install > /dev/null 2>&1

roscore > /dev/null 2>&1 &
gnome-terminal --geometry 60x16+0+0 --title="OBJECTS DETECTION" -- /bin/bash -c 'source '${pwd}'/catkin_ws/devel/setup.bash; cd '${pwd}'/catkin_ws/src; poetry run rosrun konenako node_detector.py;' &
gnome-terminal --geometry 60x16+0+359 --title="PRINTER" -- /bin/bash -c 'source '${pwd}'/catkin_ws/devel/setup.bash; cd '${pwd}'/catkin_ws/src; poetry run rosrun konenako node_printer.py;' &
gnome-terminal --geometry 60x16+625+359 --title="CAMERA" -- /bin/bash -c 'source '${pwd}'/catkin_ws/devel/setup.bash; cd '${pwd}'/catkin_ws/src; poetry run rosrun konenako node_camera.py;' &
gnome-terminal --geometry 60x16+625+0 --title="INPUT" -- /bin/bash -c 'source '${pwd}'/catkin_ws/devel/setup.bash; cd '${pwd}'/catkin_ws/src; poetry run rosrun konenako node_input.py;' &
gnome-terminal --geometry 60x16+0+700Pu --title="QR" -- /bin/bash -c 'source '${pwd}'/catkin_ws/devel/setup.bash; cd '${pwd}'/catkin_ws/src; poetry run rosrun konenako node_qr_reader.py;'


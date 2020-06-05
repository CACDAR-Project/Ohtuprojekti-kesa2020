#!/bin/sh

ros_distro=$(rosversion -d)
src_folder='rostest'
bold=$(tput bold)
normal=$(tput sgr0)
pwd=$(pwd)


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



if [ -d ${pwd}"/catkin_ws" ]
then
    echo 'Folder '${bold}'catkin_ws'${normal}' deleted!'
    rm -r catkin_ws
fi


if [ ! -d ${pwd}"/${src_folder}" ]
then
    echo 'Folder '${bold}${pwd}"/"${src_folder}${normal}' does not exist!'
    exit
fi

mkdir -p ${pwd}/catkin_ws/src
cp -r ${pwd}/${src_folder}/* ${pwd}/catkin_ws/src

echo 'Using '${ros_distro}'.'
source /opt/ros/${ros_distro}/setup.bash

poetry remove tensorflow

cd ${pwd}/catkin_ws
catkin_make

poetry add tensorflow==1.14.0

source ${pwd}/devel/setup.bash

gnome-terminal --geometry 80x24+0+0 --title="ROSCORE" -- /bin/sh -c 'cd '${pwd}' && roscore'
gnome-terminal --geometry 80x24+0+488 --title="ROSTEST" -- /bin/sh -c 'cd '${pwd}' && poetry shell && source /devel/setup.bash && cd src && rosrun rostest main.py' &
gnome-terminal --geometry 80x24+734+0 --title="MASTER" -- /bin/sh -c 'cd '${pwd}'/src && docker attach master' &
gnome-terminal --geometry 80x24+734+488 --title="ROSPRINTER" -- /bin/sh -c 'cd '${pwd}'/src && docker attach rosprinter' &

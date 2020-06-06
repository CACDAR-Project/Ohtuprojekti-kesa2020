#!/bin/sh

ros_distro=$(rosversion -d)
src_folder=rostest
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
    echo 'Folder '${bold}${pwd}'/catkin_ws'${normal}' deleted!'
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

catkin_make -C ${pwd}/catkin_ws

poetry add tensorflow==1.14.0

gnome-terminal --geometry 80x24+0+488 --title="OBJECTS DETECTION" -- /bin/bash -c 'echo '${pwd}'; source '${pwd}'/catkin_ws/devel/setup.bash; cd '${pwd}'/catkin_ws/src; rosrun rostest main.py; exec bash' &
gnome-terminal --geometry 80x24+0+488 --title="INPUT" -- /bin/bash -c 'echo '${pwd}'; source '${pwd}'/catkin_ws/devel/setup.bash; cd '${pwd}'/catkin_ws/src; rosrun rostest input.py; exec bash' &
gnome-terminal --geometry 80x24+0+488 --title="PRINTER" -- /bin/bash -c 'echo '${pwd}'; source '${pwd}'/catkin_ws/devel/setup.bash; cd '${pwd}'/catkin_ws/src; rosrun rostest printer.py; exec bash'

echo ${pwd}'/devel/setup.bash'

source ${pwd}/devel/setup.bash
roscore
#!/bin/bash
# bash is needed for easy regex tests

# This script can be used to check that you have all the required software installed
# to run the application with all the nodes. It is built upon the runner.sh script.

# If you have the the required software it will run the ros master and launch
# all the nodes that you wish to run.

# Before running this script make sure that you have read the instructions in
# README.md file found in the repository and created the ROS workspace in the
# right location.

# #######################################
# TODO: Implement option to use roslaunch
# TODO: Implement version checking for all the different required binaries
# #######################################


# Script settings
# ---------------

# When DEV is set to 0 open terminals for everything and dont suppress any output
DEV=0

# Array with all the required binaries
REQ_BIN=(poetry python3 catkin_make roscore rosparam rosrun)

# Required versions, in integers for bash compatibility
POETRY_VER=105
PYTHON_VER=370

# ROS package name
ROSPKG=konenako

# ROS distro
ROS_DISTRO=$(rosversion -d)

# Array with all the required parameters that need to be set
REQ_PARAMS=(label_file model_file video_source)
# Default values
DEF_LABEL=mscoco_complete_labels
DEF_MODEL=ssd_mobilenet_v1_1_metadata_1.tflite
DEF_VIDEO=/dev/video0

# Available nodes
NODES=(camera object_detector qr_detector printer input)

# ---------------
# Misc variables
# ---------------
CATKIN_PWD="${PWD}/catkin_ws/devel"
PROJ_PWD=$(pwd)

# Env variables

# For catkin_make
export ROS_LANG_DISABLE=genlisp:geneus:gennodejs


# Run the script
# --------------

# Source environment
echo 'Sourcing /opt/ros/'${ROS_DISTRO}'/setup.bash'
source /opt/ros/${ROS_DISTRO}/setup.bash

# Check that all required applications are installed
REQ_MET=0
echo "Checking for required software.."
echo ""
for b in ${REQ_BIN[@]}; do
    if [ -z $(which $b) ]; then
        echo "ERROR: $b is not installed!"
        REQ_MET=1
    fi
done

if [ "$REQ_MET" -ne 0 ]; then
    echo ""
    echo "You will need these programs installed before continuing."
    echo "Consult the Github page README for required versions"
    echo ""
    echo "Exiting.."
    exit 1
fi

echo "All required binaries found, continuing"
echo


# Check for virtualenv
if [ ! -z "$POETRY_ACTIVE" ] || [ ! -z "$VIRTUAL_ENV" ]; then
    echo "You are in venv! Quit venv with the command: exit. Then run this script again."
    exit 2
fi


# Check for correct versions
# Poetry
poetryver=$(poetry --version|grep -Po '(?<=version )(.+)')
poetryver=${poetryver//./}
if [ "$poetryver" -lt $POETRY_VER ]; then
    echo "You are running an outdated poetry version, please upgrade. Exiting.."
    exit 1
fi

# Python
pytver=$(poetry run python --version|grep -Po '(?<=Python )(.+)')
pytver=${pytver//./}
if [ "$pytver" -lt "$PYTHON_VER" ]; then
    echo "Your python version inside the virtual env is less than $PYTHON_VER, please upgrade. Exiting.."
    exit 1
fi

# Imported libraries, checks all required libraries
# and prints if they are missing or too old versions
# TODO: try running poetry install and check again
if ! poetry run python resources/python/libraries_versions.py; then
    exit 1
fi
echo


# Delete previous catkin workspace if it exists in project directory. 
if [ -d ${PROJ_PWD}"/catkin_ws" ]
then
    echo 'Removing previous catkin workspace from project directory!'
    rm -r ${PROJ_PWD}/catkin_ws
fi

echo 'Creating directory for new catkin workspace inside project directory.'
mkdir ${PROJ_PWD}/catkin_ws/

echo 'Copying all files from project directory to catkin workspace.'
cp -r ${PROJ_PWD}/src ${PROJ_PWD}/resources ${PROJ_PWD}/catkin_ws/

echo 'Creating catkin workspace with catkin_make'
catkin_make -C ${PROJ_PWD}/catkin_ws > /dev/null 2>&1

echo 'Sourcing '${PROJ_PWD}'/catkin_ws/devel/setup.bash'
source ${PROJ_PWD}/catkin_ws/devel/setup.bash

echo 'Installing poetry dependencies'
poetry install > /dev/null 2>&1

#Check that ROS Workspace exists
#if [ ! -d "$CATKIN_PWD" ] || [ ! -f "$CATKIN_PWD/setup.bash" ]; then
#    echo "Catkin workspace is not set up properly"
#    echo "Please consult the Github README for instructions."
#    echo "Exiting.."
#    exit 1
#fi

# (re)start roscore, requires pgrep to be installed
if [ $(pgrep roscore) ]; then
    echo "Roscore is running, restarting"
    kill $(pgrep roscore)
fi

if [ "$DEV" -eq 0 ]; then
    echo 'Opening roscore in new terminal window'
<<<<<<< HEAD
    gnome-terminal --geometry 60x16 --title="Roscore" -- /bin/bash -c 'cd '${PROJ_PWD}'; roscore; exec bash' > /dev/null 2>&1 &
=======
    gnome-terminal --geometry 60x16 --title="Roscore" -- /bin/bash -c 'cd '${PROJ_PWD}'; roscore; bash' > /dev/null 2>&1 &
>>>>>>> ee6f54cf104675771fd9eff7c76091b1d0be4412
else
    echo 'Opening roscore'
    roscore > /dev/null 2>&1 &
fi

# Wait until roscore master is running
sleep 2
echo "Master started"
echo

# Check that all required rosparams are set, if not ask user
# for value.
for p in ${REQ_PARAMS[@]}; do
    if [ ! "$(rosparam get $p)" ]; then
        # rosparam get prints an error message if param is not set..
        echo "You need to set this variable to continue:"
        
        # Suggest default values for label and model files, accept with empty input
        if [ "$p" = "label_file" ]; then
            read -p "Enter label file to use [$DEF_LABEL]: " label
            label=${label:-$DEF_LABEL}
            rosparam set $p $label
            if [ "$?" -ne 0 ]; then
                echo "Error setting parameter [$p], exiting.."
                exit 3
            fi
        elif [ "$p" = "model_file" ]; then
            read -p "Enter model file to use [$DEF_MODEL]: " model
            model=${model:-$DEF_MODEL}
            rosparam set $p $model
            if [ "$?" -ne 0 ]; then
                echo "Error setting parameter [$p], exiting.."
                exit 3
            fi
        elif [ "$p" = "video_source" ]; then
            read -p "Enter video source to use [$DEF_VIDEO]: " video
            video=${video:-$DEF_VIDEO}
            rosparam set $p $video
            if [ "$?" -ne 0 ]; then
                echo "Error setting parameter [$p], exiting.."
                exit 3
            fi
        else
            read -p "Enter value for the required parameter $p: " value
            rosparam set $p $value
            if [ "$?" -ne 0 ]; then
                echo "Error setting parameter [$p], exiting.."
                exit 3
            fi
        fi
    else
        echo "Param [$p] is set to the value: $(rosparam get $p)"
    fi
    echo
done
echo

# -------------------
# TODO: Ask user for other parameters and set them one by one until empty string..?
# -------------------


# Print all the set rosparameters, requires tr to be installed
# since many of the parameters set by roscore contains newlines
# and whitespaces.
# TODO: FIX string trimming to use only bash, maybe with parameter expansion?
if [ "$DEV" -eq 0 ] && [ "$(which tr)" ]; then
    echo "Set rosparameters:"

    for p in $(rosparam list); do
        echo "$p: $(rosparam get $p|tr -d '[:space:]')"
    done
fi
echo

# Run all the nodes
# TODO: dont open terminals for all nodes and dont append ;bash to the end so they close when DEV is not set
for node in ${NODES[@]}; do
    read -p "Do you wish to run the node: $node? [Y/y] " -n 1 -r
    echo # Newline
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Running node $node"        
<<<<<<< HEAD
        gnome-terminal --geometry 60x16 --title="$node" -- /bin/bash -c 'source '${CATKIN_PWD}'/setup.bash; cd '${PROJ_PWD}'; poetry run rosrun '${ROSPKG}' node_'${node}'.py; exec bash' /dev/null 2>&1  &
=======
        gnome-terminal --geometry 60x16 --title="$node" -- /bin/bash -c 'source '${CATKIN_PWD}'/setup.bash; cd '${PROJ_PWD}'; poetry run rosrun '${ROSPKG}' node_'${node}'.py; exec bash' > /dev/null 2>&1 &
>>>>>>> ee6f54cf104675771fd9eff7c76091b1d0be4412
    fi
    echo
done

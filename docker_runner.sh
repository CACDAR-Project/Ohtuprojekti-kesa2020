#!/bin/sh

pwd=$(pwd)
running_conts_amount=$(docker ps -q|wc -l)

# Check for open containers and if they exist, kill them
if [ $running_conts_amount -ne 0 ]; then
    echo "There was $running_conts_amount containers running."
    echo "Attempting to kill containers with the following IDs:"
    docker kill $(docker ps -q)
    if [ $(docker ps -q|wc -l) -ne 0 ]; then
        echo ""
        echo "Could not kill all of the containers, exiting.."
        exit 1
    else
        echo "All containers killed"
    fi
    echo ""
fi

echo "No containers running, continuing.."
echo ""


docker-compose build
retval=$?
if [ $retval -ne 0 ]; then
    echo "Errors running docker-compose build, exiting."
    exit $retval
fi

docker-compose up -d
retval=$?
if [ $retval -ne 0 ]; then
    echo "Errors running docker-compose up, exiting."
    exit $retval
fi


# Open new terminal windows and which attach to different containers
gnome-terminal --geometry 60x16+0+0 --title="ROSINPUT" -- /bin/sh -c 'docker attach rosinput' &
gnome-terminal --geometry 60x16+0+359 --title="ROSDETECTOR" -- /bin/sh -c 'docker attach rosdetector' &
gnome-terminal --geometry 60x16+625+0 --title="MASTER" -- /bin/sh -c 'docker attach master' &
gnome-terminal --geometry 60x16+625+359 --title="ROSPRINTER" -- /bin/sh -c 'docker attach rosprinter' &

# Open window which can be used to close all containers
gnome-terminal --geometry 60x16+0+680 --title="All containers" -- /bin/sh -c 'cd '${pwd}'; docker-compose up'


#To a avoid using sudo with docker add yourself to docker group:
#1. Create the docker group.
#       `sudo groupadd docker`
#
#2. Add your user to the docker group.
#       `sudo usermod -aG docker $USER`
#   
#3. Log out and log back in so that your group membership is re-evaluated.
#   If testing on a virtual machine, it may be necessary to restart the virtual
#   machine for changes to take effect. On a desktop Linux environment such as X 
#   Windows, log out of your session completely and then log back in.
#   On Linux, you can also run the following command to activate the changes to groups:
#       `newgrp docker`
#
#4. Verify that you can run docker commands without sudo.
#       `docker run hello-world`
#
#   This command downloads a test image and runs it in a container. When the 
#   container runs, it prints an informational message and exits.
#
#   If you initially ran Docker CLI commands using sudo before adding your user
#   to the docker group, you may see the following error, which indicates that 
#   your ~/.docker/ directory was created with incorrect permissions due to the sudo commands.
#    ```WARNING: Error loading config file: /home/user/.docker/config.json -
#       stat /home/user/.docker/config.json: permission denied```
#
#   To fix this problem, either remove the ~/.docker/ directory (it is recreated
#   automatically, but any custom settings are lost), or change its ownership
#   and permissions using the following commands:
#       `sudo chown "$USER":"$USER" /home/"$USER"/.docker -R`
#       `sudo chmod g+rwx "$HOME/.docker" -R`

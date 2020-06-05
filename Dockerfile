FROM ros:melodic-robot as rosbase
RUN apt-get update \
 && apt-get -y install \
 python3.7 \
 python3.7-dev \
 python3-pip \
 python3.7-venv \
 && rm -rf /var/lib/apt/lists/*
# These can be added to apt-get install, if needed: python-cv-bridge and ros-melodic-cv-bridge

# WORDKIR will create and set the current working directory.
# Create python virtualenv in own layer, to speed up building containers
WORKDIR /catkin_ws/src/ohtu/rostest
COPY pyproject.toml poetry.lock /catkin_ws/src/ohtu/
RUN python3.7 -m pip install --upgrade pip \
 && python3.7 -m pip install poetry \
 && poetry run pip install --upgrade pip \
 && poetry run pip install --upgrade setuptools \
 && poetry install

# Own layer for the project, NOTE: this folder (rostest/) should be changed, when the project name changes (in the repo).
# We need to create our own directory here, since docker COPY srcdic destdir actually works like COPY srcdir/* destdir
COPY rostest/ /catkin_ws/src/ohtu/rostest/
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"


# Can we omit the sourcing from these layers?
FROM rosbase as rostest
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd rostest && rosrun rostest main.py'

FROM rosbase as rosinput
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd rostest && rosrun rostest input.py'

FROM rosbase as rosprinter
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd rostest && rosrun rostest printer.py'

FROM rosbase as roscamera
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd rostest && rosrun rostest camera.py'

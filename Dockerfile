# Available official ROS-images: https://hub.docker.com/_/ros
FROM ros:melodic-ros-base as rosbase
# Disable message generators for unneeded packages (languages) specified
# in the env variable ROS_LANG_DISABLE
# This might generate a cmake warning, but it can be safely ignored:
# http://wiki.ros.org/ROS/EnvironmentVariables#ROS_LANG_DISABLE
# http://wiki.ros.org/message_generation
ENV ROS_LANG_DISABLE genlisp:geneus:gennodejs

RUN apt-get update \
 && apt-get -y install \
 python3.7 \
 python3.7-dev \
 python3-pip \
 python3.7-venv \
 # Should be replaced with a pip dependency if possible. 
 libzbar-dev \
 && rm -rf /var/lib/apt/lists/*
# These can be added to apt-get install, if needed: python-cv-bridge and ros-melodic-cv-bridge

# WORDKIR will create and set the current working directory.
# Create python virtualenv in own layer, to speed up building containers
WORKDIR /catkin_ws/src/ohtu/src
COPY pyproject.toml poetry.lock /catkin_ws/src/ohtu/
RUN python3.7 -m pip install --upgrade pip \
 && python3.7 -m pip install poetry \
 && poetry run pip install --upgrade pip \
 && poetry run pip install --upgrade setuptools \
 && poetry install


COPY resources/ /catkin_ws/src/ohtu/resources/
COPY src/ /catkin_ws/src/ohtu/src/

WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

COPY test.launch /catkin_ws/src/ohtu/


FROM rosbase as rosobjectdetector
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd src && rosrun konenako node_object_detector.py; bash'

FROM rosbase as rosinput
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd src && rosrun konenako node_input.py'

FROM rosbase as rosprinter
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd src && rosrun konenako node_printer.py'

FROM rosbase as roscamera
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd src && rosrun konenako node_camera.py'

FROM rosbase as rosqrdetector
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd src && rosrun konenako node_qr_detector.py'

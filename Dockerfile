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
# poetry needs to be run from this directory, with the pyproject.toml and poetry.lock files inside it.
WORKDIR /catkin_ws/src/ohtu
COPY . /catkin_ws/src/ohtu
RUN python3.7 -m pip install --upgrade pip \
 && python3.7 -m pip install poetry \
 && poetry run pip install --upgrade pip \
 && poetry run pip install --upgrade setuptools \
 && poetry install
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"


FROM rosbase as rostest
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd rostest && rosrun rostest main.py'

FROM rosbase as rosinput
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd rostest && rosrun rostest input.py'

FROM rosbase as rosprinter
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd rostest && rosrun rostest printer.py'

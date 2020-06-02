FROM ros:melodic-robot as rostest
RUN apt-get update
RUN apt-get -y install python3.7 python3.7-dev python3-pip python3.7-venv
RUN python3.7 -m pip install --upgrade pip
RUN python3.7 -m pip install poetry
RUN mkdir -p /catkin_ws/src/ohtu
WORKDIR /catkin_ws/src/ohtu
COPY pyproject.toml /catkin_ws/src/ohtu/
COPY poetry.lock /catkin_ws/src/ohtu/
RUN poetry run pip install --upgrade pip
RUN poetry run pip install --upgrade setuptools
RUN poetry install
COPY . /catkin_ws/src/ohtu/
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd rostest && rosrun rostest main.py'

FROM ros:melodic-robot as rosinput
RUN apt-get update
RUN apt-get -y install python3.7 python3.7-dev python3-pip python3.7-venv
RUN python3.7 -m pip install --upgrade pip
RUN python3.7 -m pip install poetry
RUN mkdir -p /catkin_ws/src/ohtu
WORKDIR /catkin_ws/src/ohtu
COPY pyproject.toml /catkin_ws/src/ohtu/
COPY poetry.lock /catkin_ws/src/ohtu/
RUN poetry run pip install --upgrade pip
RUN poetry run pip install --upgrade setuptools
RUN poetry install
COPY . /catkin_ws/src/ohtu/
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd rostest && rosrun rostest input.py'

FROM ros:melodic-robot as rosprinter
RUN apt-get update
RUN apt-get -y install python3.7 python3.7-dev python3-pip python3.7-venv
RUN python3.7 -m pip install --upgrade pip
RUN python3.7 -m pip install poetry
RUN mkdir -p /catkin_ws/src/ohtu
WORKDIR /catkin_ws/src/ohtu
COPY pyproject.toml /catkin_ws/src/ohtu/
COPY poetry.lock /catkin_ws/src/ohtu/
RUN poetry run pip install --upgrade pip
RUN poetry run pip install --upgrade setuptools
RUN poetry install
COPY . /catkin_ws/src/ohtu/
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"
CMD cd /catkin_ws/src/ohtu && poetry run /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ../../devel/setup.bash && cd rostest && rosrun rostest printer.py'

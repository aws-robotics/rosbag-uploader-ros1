FROM ros:kinetic-ros-core

RUN apt update
RUN apt install -y lcov python-pip python3-pip python-rosinstall libgtest-dev cmake python3-colcon-common-extensions

# Python 2 dependencies
RUN pip install -U coverage

# Python 3 dependencies
RUN pip3 install -U setuptools

RUN rosdep update

COPY scripts/build.sh /scripts/build.sh

ENTRYPOINT [ "/scripts/build.sh" ]
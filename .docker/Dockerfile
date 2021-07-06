ARG ROSDISTRO=foxy

FROM ros:$ROSDISTRO

ARG ROSDISTRO

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive


# Install any needed packages then clean up apt cache
# Build tools and general ROS tools
RUN apt-get update && apt-get install -y \
    software-properties-common \
    ssh \
    git \
    python3-pip \
    python3-vcstool \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget \
    #ros-$ROSDISTRO-rqt \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest 


################################################## 
######### Project workspace setup ################
##################################################
# EDIT the <package_name> and <rosinstall file> if necessary
RUN mkdir -p /dev_ws/src/<package_name>
COPY . /dev_ws/src/<package_name>/

WORKDIR /dev_ws
# # If there is a rosinstall file import the deps 
# RUN ["/bin/bash", "-c", " source /opt/ros/$ROSDISTRO/setup.bash &&\
#     vcs import src < ./src/<rosinstall file> &&\
#     vcs pull src || true"]


RUN ["/bin/bash", "-c", "source /opt/ros/$ROSDISTRO/setup.bash &&\
    colcon build --symlink-install \
    "]

CMD ["bash"]
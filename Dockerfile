FROM ros:dashing-ros-core-bionic

#Prevent user interaction while installing packages
ENV DEBIAN_FRONTEND = "noninteractive" \ 
    TERM = "xtrem"
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Source ROS2 Environement
RUN touch .bashrc
RUN echo "source /opt/ros/dashing/setup.bash" >> /root/.bashrc

#Install Dependencies
RUN apt-get update && apt-get install -y sudo \ 
    python3-pip \
    python3-colcon-common-extensions \
    python3-tk \
    ffmpeg libsm6 libxext6 \
    xauth\
    ros-dashing-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /workspace/src
ADD ./ /workspace/src/

WORKDIR /workspace/src
RUN pip3 install --upgrade pip
RUN pip3 install -r requirements.txt 

# Run Script
CMD ["python3", "./main.py"]



FROM dtam_ml:latest

# ROS Melodic Installation
# Enable accepting packages from packages.ros.org
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Set up keys
RUN apt install -y --no-install-recommends curl && \
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
# Installation
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-melodic-ros-base
# Environment setup: here we need bash not bin/sh so we specify the shell
RUN ["/bin/bash", "-c", "echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc && source /root/.bashrc"]
# Dependencies for building packages
RUN apt install -y --no-install-recommends \
	python-rosdep \
	python-rosinstall \
	python-rosinstall-generator \
	python-wstool \
	build-essential
# Initialize rosdep
RUN rosdep init && rosdep update

# Install Jetcam
COPY camera/jetcam /usr/jetcam
RUN cd /usr/jetcam && \
	python3 setup.py install
RUN pip3 install traitlets scikit-build


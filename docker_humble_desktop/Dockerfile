###########################################
# Xplore Recruitment image Build Docker
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

# Install Rover packages
RUN apt-get update && apt-get upgrade -y
RUN apt install -y curl

# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart

USER $USERNAME

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*

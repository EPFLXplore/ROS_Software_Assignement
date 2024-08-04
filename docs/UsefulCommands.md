# Useful Commands

## Linux Commands

### Navigation and File Management

- `pwd` - Print Working Directory: Displays the current directory path.
- `ls` - List: Lists files and directories in the current directory.
- `cd <directory>` - Change Directory: Navigates to the specified directory.
- `mkdir <directory>` - Make Directory: Creates a new directory.
- `rmdir <directory>` - Remove Directory: Deletes an empty directory.
- `rm <file>` - Remove: Deletes a file.
- `rm -r <directory>` - Remove Recursively: Deletes a directory and its contents.
- `cp <source> <destination>` - Copy: Copies files or directories.
- `mv <source> <destination>` - Move: Moves or renames files or directories.
- `touch <file>` - Touch: Creates an empty file or updates the timestamp of an existing file.
- `nano <file>` - Nano Editor: Opens the Nano text editor to edit a file.

### System Information

- `uname -a` - Print System Information: Displays detailed information about the system.
- `df -h` - Disk Free: Shows disk space usage in a human-readable format.
- `du -sh <directory>` - Disk Usage: Shows the size of a directory and its contents.
- `top` - Task Manager: Displays running processes and system resource usage.
- `ps aux` - Process Status: Lists all running processes.
- `free -h` - Free Memory: Displays free and used memory in a human-readable format.

### Networking

- `ifconfig` - Interface Configuration: Displays network interface configuration.
- `ping <hostname>` - Ping: Checks connectivity to a host.
- `netstat -tuln` - Network Statistics: Lists open ports and listening services.
- `curl <url>` - Curl: Transfers data from or to a server using various protocols.

### Package Management

- `sudo apt update` - Update Package List: Updates the list of available packages.
- `sudo apt upgrade` - Upgrade Packages: Upgrades all installed packages to their latest versions.
- `sudo apt install <package>` - Install Package: Installs the specified package.
- `sudo apt remove <package>` - Remove Package: Removes the specified package.
- `sudo apt autoremove` - Auto Remove: Removes unused packages.

## Docker Commands

### Basic Commands

- `docker --version` - Docker Version: Displays the installed Docker version.
- `docker pull <image>` - Pull Image: Downloads an image from Docker Hub.
- `docker build -t <name> .` - Build Image: Builds an image from a Dockerfile in the current directory and tags it with the specified name.
- `docker run <image>` - Run Container: Runs a container from the specified image.
- `docker ps` - List Containers: Lists running containers.
- `docker stop <container_id>` - Stop Container: Stops a running container.
- `docker rm <container_id>` - Remove Container: Deletes a stopped container.
- `docker image list` - List Images: Lists all Docker images on the system.
- `docker rmi <image_id>` - Remove Image: Deletes the specified image.

### Advanced Commands

- `docker exec -it <container_id> bash` - Execute Command: Opens an interactive terminal session inside a running container.
- `docker logs <container_id>` - Logs: Displays logs from a running container.
- `docker-compose up` - Docker Compose Up: Starts services defined in a `docker-compose.yml` file.
- `docker-compose down` - Docker Compose Down: Stops services defined in a `docker-compose.yml` file.
- `docker volume ls` - List Volumes: Lists all Docker volumes.
- `docker network ls` - List Networks: Lists all Docker networks.
- `docker inspect <container_id>` - Inspect Container: Displays detailed information about a container.

## ROS Commands

### Basic Commands

- `ros2 --help` - Help: Displays help information for ROS 2 commands.
- `ros2 topic list` - List Topics: Lists all active topics.
- `ros2 topic echo <topic>` - Echo Topic: Displays messages published to the specified topic.
- `ros2 topic pub <topic> <msg_type> <msg>` - Publish Topic: Publishes a message to a topic.
- `ros2 node list` - List Nodes: Lists all active nodes.
- `ros2 node info <node_name>` - Node Info: Displays information about a specific node.

### Package and Workspace Management

- `ros2 pkg create <package_name>` - Create Package: Creates a new ROS 2 package.
- `colcon build` - Build Workspace: Builds the ROS 2 workspace.
- `source <workspace>/install/setup.bash` - Source Workspace: Sources the setup file to overlay the workspace on the environment.

### Launching and Running Nodes

- `ros2 run <package_name> <executable>` - Run Node: Runs a node from a specified package.
- `ros2 launch <package_name> <launch_file>` - Launch: Launches nodes and configurations specified in a launch file.

### Service and Parameter Commands

- `ros2 service list` - List Services: Lists all active services.
- `ros2 service call <service> <srv_type> <args>` - Call Service: Calls a service with specified arguments.
- `ros2 param list` - List Parameters: Lists all parameters for a node.
- `ros2 param set <node> <param_name> <value>` - Set Parameter: Sets a parameter value for a node.

### Additional Tools

- `rviz2` - RViz: Launches the RViz visualization tool.
- `rqt` - RQt: Launches the RQt graphical user interface tool.

# Level 3: Custom Services and Integration

Welcome to Level 3! In this level, you'll learn how to create a custom service in ROS that interacts with a subscriber. This will help you understand how to implement more complex communication patterns in ROS, which are essential for building robust robotic applications.

## Objectives

- Create a custom service that triggers a Console print and returns a Result
- Couple the service with the subscriber

### Approximate Time: 2h30

## Scenario: Validating Rover Position

Imagine you are tasked with ensuring that the rover stays within safe operating boundaries. You will create a service that checks if a given position is within the allowed bounds and returns a result. Additionally (BONUS), the service will provide a suggestion to move further away from the boundaries if needed.

### Client

The client will send a request to check if a new position is allowed, i.e., within the bounds `[[-100, 100], [-100, 100]]`.

### Server

The server will respond with a boolean indicating whether the position is allowed and (BONUS) a string suggesting a move to get further away from the boundaries if the position is near the edges (< 5).

## Step-by-Step Instructions

### Step 1: Create a ROS Service

1. **Create the service definition**:

   Inside the `custom_msg` package, create a new directory called `srv` and define a new service file named `CheckPosition.srv`.

   Add the following content to `CheckPosition.srv`:

   ```plaintext
   float64 x
   float64 z
   float64 ry
   ---
   bool is_allowed
   string suggestion
   ```

   This defines a service that takes three `float64` inputs (x, z and ry) and returns a boolean (`is_allowed`) and a string (`suggestion`).

2. **Update `CMakeLists.txt`**:

   In the `custom_msg` package directory, open `CMakeLists.txt` and add the following lines to find the necessary packages and generate the service files:

   ```cmake
   find_package(rosidl_default_generators REQUIRED)
   rosidl_generate_interfaces(${PROJECT_NAME}
     "srv/CheckPosition.srv"
   )
   ```

### Step 2: Create the Service Server Node

1. **Create the server script**:

   Inside the `rover_commands/rover_commands` folder, create a new file named `check_position_server.py`.

2. **Edit the server script**:

   Open `check_position_server.py` with a text editor and add the following code:

   ```python
   import rclpy
   from rclpy.node import Node
   from custom_msg.srv import CheckPosition

   class CheckPositionServer(Node):

       def __init__(self):
           super().__init__('check_position_server')
           # TODO: Create a server of type CheckPosition.srv that calls check_position_callback at each request.
           # Your code here

           self.get_logger().info('Service server has been started.')

       def check_position_callback(self, request, response):
           # TODO: Get the inputs from the request, and process them to check the boundaries
           # Your code here


           # TODO - BONUS: Try to give a recommendation of which move would help moving further away from the boundaries if you are too close to a boundary.
           # Your code here


           self.get_logger().info(f'Received request: x={x}, z={z}')
           self.get_logger().info(f'Response: is_allowed={response.is_allowed}, suggestion="{response.suggestion}"')
           return response

   def main(args=None):
       rclpy.init(args=args)
       node = CheckPositionServer()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

   Now your turn to complete it! Use online resources such as [ROS doc](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html).

3. **Update the `setup.py`**:

   In the `rover_commands` directory, open the `setup.py` file and add the following entry to the `console_scripts` list:

   ```python
   entry_points={
       'console_scripts': [
           'publisher = rover_commands.publisher:main',
           'subscriber = rover_commands.subscriber:main',
           'check_position_server = rover_commands.check_position_server:main',
       ],
   },
   ```

   This tells ROS to register the `check_position_server` script as an executable node.

### Step 3: Integrate the Service with the Subscriber

1.  **Update the subscriber script**:

    Edit the `subscriber.py` script to include the service client functionality:

    ```python
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from rover_commands.srv import CheckPosition

    class TrajectorySubscriber(Node):

        def __init__(self):
            super().__init__('trajectory_subscriber')
            # TODO: Create a subscriber of type Twist, that calls listener_callback
            # Your code here

            # TODO: Create a server client of type CheckPosition
            # Your code here

            self.get_logger().info('Subscriber node has been started.')
            self.position = {'x': 0.0, 'z': 0.0, 'ry': 0.0}

        def send_request(self, x, z):
             # TODO: Send the request to the server asynchronously
             # Your code here

        def listener_callback(self, msg):
            # TODO: Interpret the received commands and log the result using self.get_logger().info()
            # Your code here

            # TODO: Call send_request and wait for the result. Then, update the position of your virtual rover if allowed
             # Your code here

            self.get_logger().info(f'Response: is_allowed={is_allowed}, suggestion="{suggestion}"')
            self.get_logger().info(f'New Position: {self.position}')

    def main(args=None):
        rclpy.init(args=args)
        node = TrajectorySubscriber()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

### Step 5: Build and Run the Package

1. **Build the package**:

   In the `dev_ws` directory, build the package:

   ```sh
   cd ~/dev_ws/
   colcon build
   ```

   This command compiles the package and sets up the necessary environment.

2. **Source the setup file**:

   After building, source the setup file to overlay the workspace on your environment:

   ```sh
   . install/setup.bash
   ```

   This command sets up the environment variables needed to run the nodes. You will have to execute this command on every terminal you open to help it find your nodes.

3. **Run the server node**:

   In one terminal inside the Docker container, run the service server node:

   ```sh
   ros2 run rover_commands check_position_server
   ```

   This starts the `CheckPositionServer` node, which listens for position check requests and responds accordingly.

4. **Run the subscriber node**:

   Open a new terminal inside the Docker container and run the subscriber node:

   ```sh
   docker exec -it base_humble_desktop bash
   . install/setup.bash
   ros2 run rover_commands subscriber
   ```

   This starts the `TrajectorySubscriber` node, which listens for `Twist` messages on the `trajectory` topic, interprets the commands, and sends position check requests to the service server.

### Example Run

1. **Start the Server Node**:

   In the first terminal, the server node will start and wait for position check requests.

2. **Monitor the Subscriber Node**:

   In the second terminal, the subscriber node will interpret the `Twist` messages, update the position, and send position check requests to the server. The server will respond with whether the position is allowed and a suggestion if necessary.

   Example Output:

   ```sh
   [INFO] [subscriber]: Go Forward
   [INFO] [subscriber]: New Position: {'x': 1.0, 'z': 0.0, 'ry': 0.0}
   [INFO] [subscriber]: Response: is_allowed=True, suggestion="Position is within the safe zone"
   ```

By completing these steps, you have created a ROS package with a custom service that interacts with a subscriber to validate the rover's position.

Congratulations on completing Level 3! You have now mastered the basics of creating and integrating custom services in ROS.

If you arrived here, **we really want you to be in our team**!

![gif the voice](https://media3.giphy.com/media/PSZWmbhGaDX8Y/giphy.gif?cid=6c09b9521mlesjs7iyuqj9ospdfdmtu1zj7u9gzqo7lnzcep&ep=v1_internal_gif_by_id&rid=giphy.gif&ct=g)

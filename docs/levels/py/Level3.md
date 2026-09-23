# Level 3: Package Infrastructure and Multiple Nodes

Welcome to Level 3! In this level, you'll take what you accomplished in the previous level and extend it to create a ROS package that controls a virtual rover, integrating rover drift correction using sensors.

## Objectives

- Extend your package to multiple publishers and subscribers
- Experience a bit merging data from different topics

## Scenario: Controlling the Rover

![Nodes Graph](../../level3.png)

In addition to what you did in level 2, you will now have to work with the following:
- `TrajectoryPublisher` should now be called `GamepadNode`, and should publish `Twist` messages to the `input_cmd` topic;
- The new `SensorNode` will publish `Twist` messages to the `correction_cmd` topic, modeling the correction your rover should apply, provided by imaginary sensors;
- The new `ProcessNode` will listen on both the `input_cmd` and `correction_cmd` topics, and publish the resulting `Twist`  representing the real position of the rover to `gps_pos`;
- The new `GPSNode` will listen on the `gps_pos` topic, and print the resulting position to the screen.

> [!NOTE]
>
> The `ProcessNode` calculation should work the following way:
>
> - Keep a local `Twist` of the current position.
> - Whenever you get a `Twist` from `input_cmd` or `correction_cmd`, add it to the current position.
> - Publish the current position to `gps_pos` every second.


## Step-by-Step Instructions

This level will be way less guided than the previous ones.

Here are the base codes for some files:

> `rover_commands/src/gamepad.py`

The `publisher.py` code from level 2, renaming needed things :)

> `rover_commands/src/sensor.py`

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor')
        # TODO: Create a publisher of type Twist
		# Your code here...
		
        self.start_time = self.get_clock().now()

		# TODO: Make the `publish_correction` be called every second
		# Hint : the ROS documentation may have something for you
        # Your code here...


        self.get_logger().info('Sensor node has been started.')

    def publish_correction(self):
        msg = Twist()

        msg.linear.x = math.sin(t)
        msg.linear.z = math.cos(t)
        msg.angular.y = t

        # TODO: Publish 'msg' to the 'correction_cmd' topic
        # Your code here...
        
        self.get_logger().info(f'Published correction: x={msg.linear.x:.3f}, z={msg.linear.z:.3f}, ry={msg.angular.y:.3f}.')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

> `rover_commands/src/process.py`

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ProcessNode(Node):

    def __init__(self):
        super().__init__('process_node')

        # TODO: Create a subscriber on the 'input_cmd' topic
        # Your code here...

        # TODO: Create a subscriber on the 'correction_cmd' topic
        # Your code here...

        # TODO: Create a publisher of type Twist on the 'gps_pos' topic
        # Your code here...

        self.get_logger().info('Process node has been started.')

    # TODO: Add the necessary functions, make sure to read the note on the process node
    # Your code here...

def main(args=None):
    rclpy.init(args=args)
    node = ProcessNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

> `rover_commands/src/gps.py`

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node')

        # TODO: Create a subscriber on the 'gps_pos' topic
        # Your code here...

        self.get_logger().info('GPS node has been started.')

    # TODO: Add necessary functions here
    # Your code here...

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Now, good luck, and don't forget, the [ROS documentation](https://docs.ros.org/en/humble/index.html) is your friend, and you can always ask questions.

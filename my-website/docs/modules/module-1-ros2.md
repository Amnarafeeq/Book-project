# Module 1: The Robotic Nervous System (ROS 2)

## Introduction
ROS 2 (Robot Operating System 2) serves as the nervous system of robotic applications, providing the communication infrastructure that allows different components of a robot to work together seamlessly. In this module, you will learn the fundamentals of ROS 2 and how it enables complex robotic behaviors.

## Skills You Will Learn
- ROS 2 architecture and concepts (Nodes, Topics, Services, Actions)
- Creating and managing ROS 2 packages
- Implementing publishers and subscribers
- Working with services and actions
- Using ROS 2 tools for debugging and visualization
- Building distributed robotic systems

## Tools You Will Use
- ROS 2 Humble Hawksbill (or latest LTS)
- ROS 2 development tools (ros2 run, ros2 launch, rqt, rviz2)
- Python and C++ for ROS 2 development
- Gazebo simulation environment
- Robot description format (URDF)

## Example Code

```python
# Example ROS 2 publisher node
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Key Concepts

### Nodes
Nodes are the fundamental building blocks of a ROS 2 system. Each node performs a specific task and communicates with other nodes through topics, services, or actions.

### Topics and Messages
Topics enable asynchronous communication between nodes using a publish-subscribe pattern. Messages are the data structures that are passed between nodes.

### Services and Actions
Services provide synchronous request-response communication, while actions offer asynchronous communication with feedback for long-running tasks.

## Diagrams
![ROS 2 Architecture](/img/ros2-architecture.png)

:::note
ROS 2 uses DDS (Data Distribution Service) as its underlying communication middleware, providing reliable, real-time communication between nodes.
:::

:::tip
When designing your robotic system, consider which communication pattern (topic, service, or action) best fits your use case.
:::

## Next Steps
After completing this module, you will have a solid understanding of ROS 2 fundamentals and be ready to explore digital twin technologies in Module 2.
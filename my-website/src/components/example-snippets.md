# Example Code Snippets for Physical AI & Humanoid Robotics Docusaurus Components

## Using the ModuleCard Component

```md
import ModuleCard from '@site/src/components/ModuleCard';

<ModuleCard
  title="Module 1: The Robotic Nervous System (ROS 2)"
  description="Learn the fundamentals of ROS 2 and how it enables complex robotic behaviors."
  icon="🤖"
  link="/docs/modules/module-1-ros2"
  skills={[
    "ROS 2 architecture and concepts",
    "Creating and managing ROS 2 packages",
    "Implementing publishers and subscribers"
  ]}
  tools={[
    "ROS 2 Humble Hawksbill",
    "Python and C++ for ROS 2 development",
    "Gazebo simulation environment"
  ]}
/>
```

## Using the CalloutBox Component

```md
import CalloutBox from '@site/src/components/CalloutBox';

<CalloutBox type="note" title="Important Note">
  ROS 2 uses DDS (Data Distribution Service) as its underlying communication middleware, providing reliable, real-time communication between nodes.
</CalloutBox>

<CalloutBox type="tip" title="Pro Tip">
  When designing your robotic system, consider which communication pattern (topic, service, or action) best fits your use case.
</CalloutBox>

<CalloutBox type="caution" title="Warning">
  Always ensure proper safety measures when deploying robotic systems in real-world environments.
</CalloutBox>
```

## Using the CodeBlock Component

```md
import CodeBlock from '@site/src/components/CodeBlock';

<CodeBlock language="python">
{`# Example ROS 2 publisher node
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
        self.i += 1`}
</CodeBlock>
```

## Using Components Together

```md
import ModuleCard from '@site/src/components/ModuleCard';
import CalloutBox from '@site/src/components/CalloutBox';
import CodeBlock from '@site/src/components/CodeBlock';

# Module Overview

<ModuleCard
  title="The Digital Twin (Gazebo & Unity)"
  description="Create simulation environments for testing and training robotic systems."
  icon="🎮"
  link="/docs/modules/module-2-gazebo-unity"
  skills={[
    "Creating and configuring robot models in Gazebo",
    "Building 3D environments in Unity for robotics",
    "Integrating Unity with ROS 2"
  ]}
  tools={[
    "Gazebo Garden or Fortress",
    "Unity 3D",
    "Robot Operating System 2 (ROS 2)"
  ]}
/>

<CalloutBox type="info" title="Simulation Benefits">
  Simulation environments allow for rapid prototyping and testing of robotic algorithms without the constraints and risks associated with physical hardware.
</CalloutBox>

<CodeBlock language="xml">
{`<!-- Example URDF robot model for Gazebo -->
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>`}
</CodeBlock>
```
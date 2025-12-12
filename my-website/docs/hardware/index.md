# Hardware Requirements

Building physical AI and humanoid robotic systems requires specialized hardware for both development and deployment. This section outlines the recommended hardware configurations for different aspects of the course.

## Digital Twin Workstation

For simulation and development work, we recommend a workstation with:

- **CPU**: AMD Ryzen 7 5800X or Intel i7-12700K
- **GPU**: NVIDIA RTX 3080/4080 or RTX A4000/A5000 for professional applications
- **RAM**: 32GB DDR4-3200 or higher
- **Storage**: 1TB NVMe SSD for fast loading of simulation environments
- **OS**: Ubuntu 22.04 LTS or Windows 11 with WSL2

### Recommended Configurations

#### Student Configuration
- CPU: AMD Ryzen 5 5600X or Intel i5-12400F
- GPU: NVIDIA RTX 3060 or RTX 4060
- RAM: 16GB DDR4-3200
- Storage: 500GB NVMe SSD
- Cost: ~$1,200-1,500

#### Professional Configuration
- CPU: AMD Ryzen 9 5900X or Intel i9-12900K
- GPU: NVIDIA RTX 4090 or RTX A6000
- RAM: 64GB DDR4-3600
- Storage: 2TB NVMe SSD + 4TB HDD for assets
- Cost: ~$3,500-5,000

## Jetson Edge Kit

For edge AI deployment on robots, we recommend:

- **NVIDIA Jetson Orin Nano Developer Kit** (recommended for learning)
  - 1024 CUDA cores, 40 TOPS AI performance
  - 4GB/8GB LPDDR5 memory
  - 16GB eMMC storage
  - Power: 7-15W

- **NVIDIA Jetson Orin NX Developer Kit** (recommended for advanced projects)
  - 1024 CUDA cores, 100 TOPS AI performance
  - 8GB LPDDR5 memory
  - 16GB eMMC storage
  - Power: 15-25W

## Robot Lab Options

### Option A: Educational Robot Platform
- **TurtleBot 4** or **Clearpath Jackal**
  - ROS 2 native support
  - Modular sensors and computing options
  - Educational curriculum included
  - Price: ~$4,000-8,000

### Option B: Custom Humanoid Platform
- **Unitree Go1** or **AlienGo** quadruped
  - Advanced control capabilities
  - High payload capacity
  - ROS 2 integration
  - Price: ~$20,000-40,000

### Option C: Research Platform
- **ANYmal** or custom humanoid platform
  - Maximum flexibility and capability
  - Advanced sensors and actuators
  - Custom development required
  - Price: ~$50,000-150,000

## Cloud-Native Lab (AWS/Omniverse)

For teams without access to physical hardware:

- **AWS RoboMaker** for cloud robotics simulation
- **NVIDIA Omniverse** for photorealistic simulation
- **AWS EC2 G5 instances** with RTX A4000/A5000 GPUs
- **Docker containers** for reproducible environments

### Cloud Configuration
- GPU Instance: g5.2xlarge or larger
- Storage: 100GB+ SSD
- Network: High bandwidth for real-time simulation
- Cost: ~$1-3/hour depending on configuration

## Hardware Architecture Diagrams

### Development Setup
```
[Workstation] ←→ [Simulation Environment] ←→ [Robot Control Stack]
     ↓                    ↓                          ↓
[ROS 2 Nodes] → [Gazebo/Unity] → [Isaac Perception] → [VLA System]
```

### Deployment Setup
```
[Jetson Edge AI] → [Robot Hardware] → [Sensors & Actuators]
      ↑                  ↑                   ↑
[ROS 2 Bridge] ← [Motor Controllers] ← [IMU, Cameras, LIDAR]
```

:::note
Hardware requirements may vary based on specific project needs. The configurations listed here are recommendations based on typical use cases in the course.
:::

:::tip
Start with simulation and virtual environments before investing in physical hardware. This allows you to test algorithms and systems before deployment.
:::
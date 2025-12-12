---
title: Sensor Simulation in Physics Environment
sidebar_position: 7
---

# Section 6: Sensor Simulation in Physics Environment

## Learning Objectives
- Integrate LiDAR sensors with physics-based occlusion
- Simulate depth camera data with realistic noise models
- Implement IMU sensors with physics-based acceleration
- Validate sensor data against ground truth

## Content Outline

### LiDAR Simulation with Ray Casting and Physics Intersection
LiDAR (Light Detection and Ranging) sensors are crucial for robotics perception and navigation:

#### Ray Casting Implementation
- Cast rays from sensor origin in multiple directions
- Detect intersections with physical objects in the environment
- Calculate distances to nearest obstacles
- Account for sensor range limitations and resolution

#### Physics Integration
- Use collision meshes for accurate ray-object intersection
- Consider sensor mounting position and orientation
- Handle occlusions and self-occlusions
- Account for material properties affecting detection

#### Performance Considerations
- Limit number of rays for real-time performance
- Use spatial partitioning for efficient ray-object intersection
- Implement variable resolution based on distance

### Depth Camera Implementation with Realistic Distortion
Depth cameras provide 3D information about the environment:

#### Depth Calculation
- Calculate depth from camera center to objects
- Apply perspective projection model
- Include realistic noise models
- Handle depth discontinuities at object boundaries

#### Noise Modeling
- Gaussian noise for random variations
- Depth-dependent noise (higher noise at greater distances)
- Quantization effects in digital sensors
- Temporal noise patterns

#### Distortion Correction
- Radial distortion (barrel/pincushion)
- Tangential distortion
- Principal point offset
- Fisheye distortion for wide-angle cameras

### IMU Sensor Modeling with Acceleration and Rotation
Inertial Measurement Units provide motion and orientation data:

#### Physical Principles
- Accelerometers measure linear acceleration and gravity
- Gyroscopes measure angular velocity
- Magnetometers measure magnetic field (for heading)
- Combined to estimate position, velocity, and orientation

#### Physics Integration
- Acceleration from rigid body motion
- Gravity compensation in local frame
- Angular velocity from rotational motion
- Integration errors and drift modeling

#### Noise and Bias Modeling
- Bias drift over time
- Random walk characteristics
- Scale factor errors
- Cross-axis sensitivity

### Sensor Fusion in Simulated Environments
Combining multiple sensors for improved perception:

#### Data Association
- Matching features across sensors
- Handling different update rates
- Time synchronization
- Handling sensor failures

#### Fusion Algorithms
- Kalman filtering approaches
- Particle filtering for non-linear systems
- Sensor calibration and extrinsic parameters
- Uncertainty propagation

## Diagram/Code Notes
```python
# Placeholder for LiDAR simulation code
# Example of depth camera noise modeling
# IMU sensor data generation code
```

## Implementation Example: LiDAR Sensor Simulation

```python
import numpy as np
import math
from typing import List, Tuple, Optional

class LiDARSensor:
    """Simulates a 2D or 3D LiDAR sensor with physics-based ray casting"""

    def __init__(self,
                 num_beams: int = 360,
                 max_range: float = 10.0,
                 min_range: float = 0.1,
                 fov: float = 360.0,  # Field of view in degrees
                 noise_std: float = 0.01,  # Standard deviation of noise
                 resolution: float = 0.01,  # Angular resolution in degrees
                 position: Tuple[float, float, float] = (0, 0, 0),
                 rotation: Tuple[float, float, float] = (0, 0, 0)):

        self.num_beams = num_beams
        self.max_range = max_range
        self.min_range = min_range
        self.fov = fov
        self.noise_std = noise_std
        self.position = np.array(position, dtype=float)
        self.rotation = np.array(rotation, dtype=float)

        # Calculate angles for each beam
        self.angles = np.linspace(0, math.radians(fov), num_beams, endpoint=False)

        # For 3D LiDAR, we might have multiple layers
        self.elevations = [0.0]  # 2D LiDAR has all beams at same elevation

    def simulate_scan(self, environment_map: np.ndarray,
                     robot_pose: Tuple[float, float, float] = (0, 0, 0)) -> List[float]:
        """
        Simulate LiDAR scan in a 2D environment map

        Args:
            environment_map: 2D occupancy grid (0 = free, 1 = occupied)
            robot_pose: (x, y, theta) in world coordinates

        Returns:
            List of distances for each beam
        """
        robot_x, robot_y, robot_theta = robot_pose
        sensor_x = robot_x + self.position[0]
        sensor_y = robot_y + self.position[1]

        # Convert angles to world frame
        world_angles = self.angles + robot_theta

        ranges = []

        for angle in world_angles:
            # Ray casting from sensor position
            range_val = self._ray_cast(environment_map,
                                     sensor_x, sensor_y,
                                     angle, robot_pose)

            # Add noise
            if range_val < self.max_range:
                range_val += np.random.normal(0, self.noise_std)
                # Ensure range is within valid bounds
                range_val = max(self.min_range, min(range_val, self.max_range))

            ranges.append(range_val)

        return ranges

    def _ray_cast(self,
                  env_map: np.ndarray,
                  start_x: float,
                  start_y: float,
                  angle: float,
                  robot_pose: Tuple[float, float, float]) -> float:
        """Perform ray casting to find distance to obstacle"""
        # Convert to map coordinates
        map_start_x = int(start_x / self.resolution)
        map_start_y = int(start_y / self.resolution)

        # Calculate ray direction
        dx = math.cos(angle)
        dy = math.sin(angle)

        # Step along the ray
        step_size = 0.05  # 5cm steps
        distance = 0.0

        while distance < self.max_range:
            # Calculate current position
            current_x = start_x + dx * distance
            current_y = start_y + dy * distance

            # Convert to map coordinates
            map_x = int(current_x / self.resolution)
            map_y = int(current_y / self.resolution)

            # Check bounds
            if (map_x < 0 or map_x >= env_map.shape[1] or
                map_y < 0 or map_y >= env_map.shape[0]):
                return self.max_range  # Out of bounds

            # Check if hit obstacle
            if env_map[map_y, map_x] > 0.5:  # Threshold for occupied
                return distance

            distance += step_size

        return self.max_range  # No obstacle found within range

    def get_point_cloud(self, ranges: List[float]) -> List[Tuple[float, float]]:
        """Convert range measurements to 2D point cloud"""
        points = []

        for i, r in enumerate(ranges):
            if r < self.max_range and r > self.min_range:
                angle = self.angles[i]
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))

        return points

# Example usage
if __name__ == "__main__":
    # Create a simple environment (2D occupancy grid)
    env_map = np.zeros((200, 200))  # 200x200 grid, 10x10m area with 5cm resolution

    # Add some obstacles
    env_map[100:110, 50:150] = 1  # Wall
    env_map[50:80, 150:160] = 1   # Box

    # Create LiDAR sensor
    lidar = LiDARSensor(num_beams=360, max_range=10.0, noise_std=0.02)

    # Simulate scan from robot position (5, 5) with 0 rotation
    ranges = lidar.simulate_scan(env_map, robot_pose=(5.0, 5.0, 0.0))

    print(f"LiDAR scan completed. First 10 ranges: {ranges[:10]}")
    print(f"Number of valid measurements: {sum(1 for r in ranges if r < lidar.max_range)}")
```

## Implementation Example: Depth Camera Simulation

```python
import numpy as np
import cv2
from typing import Tuple, Optional

class DepthCamera:
    """Simulates a depth camera with realistic noise and distortion"""

    def __init__(self,
                 width: int = 640,
                 height: int = 480,
                 fov: float = 60.0,  # Field of view in degrees
                 min_depth: float = 0.1,  # Minimum detectable depth (m)
                 max_depth: float = 10.0,  # Maximum detectable depth (m)
                 noise_params: dict = None):

        self.width = width
        self.height = height
        self.fov = fov
        self.min_depth = min_depth
        self.max_depth = max_depth

        # Calculate focal length from FOV
        self.focal_length = width / (2 * np.tan(np.radians(fov) / 2))
        self.cx = width / 2  # Principal point x
        self.cy = height / 2  # Principal point y

        # Camera intrinsic matrix
        self.K = np.array([[self.focal_length, 0, self.cx],
                          [0, self.focal_length, self.cy],
                          [0, 0, 1]])

        # Default noise parameters
        self.noise_params = noise_params or {
            'gaussian_std': 0.01,  # Base noise standard deviation
            'depth_dependent_factor': 0.001,  # Noise increases with depth
            'quantization_levels': 1000  # Number of discrete depth levels
        }

    def simulate_depth_image(self,
                           depth_map: np.ndarray,
                           add_noise: bool = True) -> np.ndarray:
        """
        Simulate depth camera output from a ground truth depth map

        Args:
            depth_map: Ground truth depth image (H, W)
            add_noise: Whether to add realistic noise

        Returns:
            Simulated depth image with noise
        """
        # Start with ground truth
        simulated_depth = depth_map.copy().astype(np.float32)

        if add_noise:
            # Add Gaussian noise
            gaussian_noise = np.random.normal(0, self.noise_params['gaussian_std'], simulated_depth.shape)

            # Add depth-dependent noise (noise increases with distance)
            depth_factor = self.noise_params['depth_dependent_factor'] * simulated_depth
            depth_noise = np.random.normal(0, depth_factor)

            # Combine noises
            total_noise = gaussian_noise + depth_noise

            # Apply noise to depth values
            simulated_depth += total_noise

            # Apply quantization (simulates digital sensor limitations)
            max_val = self.noise_params['quantization_levels']
            simulated_depth = np.round(simulated_depth * max_val) / max_val

        # Apply depth limits
        simulated_depth = np.clip(simulated_depth, self.min_depth, self.max_depth)

        # Set invalid regions (where ground truth was 0) to max_depth
        simulated_depth[depth_map == 0] = 0  # Mark as invalid

        return simulated_depth

    def add_distortion(self, points: np.ndarray) -> np.ndarray:
        """
        Apply radial and tangential distortion to points
        points: array of shape (N, 2) with normalized coordinates
        """
        # Distortion coefficients (typical values for RGB-D cameras)
        k1, k2, p1, p2, k3 = 0.1, -0.05, 0.001, 0.001, 0.001

        # Normalize points by principal point and focal length
        x = (points[:, 0] - self.cx) / self.focal_length
        y = (points[:, 1] - self.cy) / self.focal_length

        # Calculate distortion
        r2 = x*x + y*y
        r4 = r2*r2
        r6 = r4*r2

        radial_distortion = 1 + k1*r2 + k2*r4 + k3*r6
        tangential_distortion_x = 2*p1*x*y + p2*(r2 + 2*x*x)
        tangential_distortion_y = p1*(r2 + 2*y*y) + 2*p2*x*y

        # Apply distortion
        x_distorted = x * radial_distortion + tangential_distortion_x
        y_distorted = y * radial_distortion + tangential_distortion_y

        # Convert back to pixel coordinates
        distorted_points = np.column_stack([
            x_distorted * self.focal_length + self.cx,
            y_distorted * self.focal_length + self.cy
        ])

        return distorted_points

    def project_to_3d(self, depth_image: np.ndarray) -> np.ndarray:
        """
        Convert depth image to 3D point cloud

        Args:
            depth_image: (H, W) depth values

        Returns:
            (H, W, 3) array of 3D coordinates
        """
        h, w = depth_image.shape

        # Create coordinate grids
        x_coords, y_coords = np.meshgrid(np.arange(w), np.arange(h))

        # Convert to normalized coordinates
        x_norm = (x_coords - self.cx) / self.focal_length
        y_norm = (y_coords - self.cy) / self.focal_length

        # Calculate 3D coordinates
        z_coords = depth_image
        x_coords_3d = x_norm * z_coords
        y_coords_3d = y_norm * z_coords

        # Stack to create 3D points
        points_3d = np.stack([x_coords_3d, y_coords_3d, z_coords], axis=-1)

        return points_3d

# Example usage
if __name__ == "__main__":
    # Create a sample depth map (simulating a simple scene)
    depth_map = np.ones((480, 640)) * 5.0  # 5m default depth

    # Add a "wall" at 2m depth in the center
    depth_map[200:300, 250:400] = 2.0

    # Add some noise to make it more realistic
    noise = np.random.normal(0, 0.01, depth_map.shape)
    depth_map += noise

    # Create depth camera
    depth_cam = DepthCamera(width=640, height=480, fov=60.0)

    # Simulate depth image
    simulated_depth = depth_cam.simulate_depth_image(depth_map)

    print(f"Depth camera simulation completed.")
    print(f"Original depth range: {depth_map.min():.3f} - {depth_map.max():.3f}m")
    print(f"Simulated depth range: {simulated_depth[simulated_depth > 0].min():.3f} - {simulated_depth[simulated_depth > 0].max():.3f}m")
```

## Implementation Example: IMU Sensor Simulation

```python
import numpy as np
from typing import Tuple, List
import math

class IMUSensor:
    """Simulates an IMU sensor with accelerometer, gyroscope, and magnetometer"""

    def __init__(self,
                 accelerometer_noise_density: float = 0.017,  # (m/s^2)/sqrt(Hz)
                 gyroscope_noise_density: float = 0.001,     # (rad/s)/sqrt(Hz)
                 magnetometer_noise_density: float = 0.1,    # micro-Tesla/sqrt(Hz)
                 accelerometer_bias_std: float = 0.01,       # m/s^2
                 gyroscope_bias_std: float = 0.0001,         # rad/s
                 sample_rate: float = 100.0):                # Hz

        self.accel_noise_density = accelerometer_noise_density
        self.gyro_noise_density = gyroscope_noise_density
        self.mag_noise_density = magnetometer_noise_density
        self.accel_bias_std = accelerometer_bias_std
        self.gyro_bias_std = gyroscope_bias_std
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate

        # Initialize bias states
        self.accel_bias = np.random.normal(0, self.accel_bias_std, 3)
        self.gyro_bias = np.random.normal(0, self.gyro_bias_std, 3)

        # Magnetometer reference field (Earth's magnetic field)
        self.mag_reference = np.array([22.0, 0.0, 44.0])  # micro-Tesla (approximate for location)

    def simulate_reading(self,
                        true_accel: np.ndarray,
                        true_gyro: np.ndarray,
                        true_orientation: np.ndarray,
                        time_step: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Simulate IMU readings based on true values

        Args:
            true_accel: True linear acceleration in sensor frame (3,)
            true_gyro: True angular velocity in sensor frame (3,)
            true_orientation: True orientation quaternion (4,)
            time_step: Time since last reading

        Returns:
            (accelerometer, gyroscope, magnetometer) readings
        """
        # Update biases (random walk)
        self._update_biases(time_step)

        # Calculate noise standard deviations
        accel_noise_std = self.accel_noise_density / np.sqrt(self.dt)
        gyro_noise_std = self.gyro_noise_density / np.sqrt(self.dt)
        mag_noise_std = self.mag_noise_density / np.sqrt(self.dt)

        # Add noise and bias to true values
        accel_reading = true_accel + self.accel_bias + np.random.normal(0, accel_noise_std, 3)
        gyro_reading = true_gyro + self.gyro_bias + np.random.normal(0, gyro_noise_std, 3)

        # Transform magnetic field to sensor frame and add noise
        mag_reading = self._transform_magnetic_field(true_orientation) + np.random.normal(0, mag_noise_std, 3)

        return accel_reading, gyro_reading, mag_reading

    def _update_biases(self, time_step: float):
        """Update bias states using random walk model"""
        # Random walk for biases (bias drift)
        accel_bias_drift = np.random.normal(0, self.accel_bias_std * np.sqrt(time_step), 3)
        gyro_bias_drift = np.random.normal(0, self.gyro_bias_std * np.sqrt(time_step), 3)

        self.accel_bias += accel_bias_drift
        self.gyro_bias += gyro_bias_drift

    def _transform_magnetic_field(self, orientation: np.ndarray) -> np.ndarray:
        """Transform magnetic field from world to sensor frame"""
        # Convert quaternion to rotation matrix
        R = self._quaternion_to_rotation_matrix(orientation)

        # Transform reference magnetic field
        mag_world_frame = self.mag_reference
        mag_sensor_frame = R.T @ mag_world_frame

        return mag_sensor_frame

    def _quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """Convert quaternion to rotation matrix"""
        w, x, y, z = q

        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])

        return R

class IMUSimulator:
    """Complete IMU simulation with physics integration"""

    def __init__(self, imu_config: dict = None):
        self.imu = IMUSensor(**(imu_config or {}))
        self.time = 0.0

        # State variables
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.angular_velocity = np.zeros(3)

    def step(self, external_force: np.ndarray = None,
             external_torque: np.ndarray = None,
             dt: float = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Step the simulation and return IMU readings

        Args:
            external_force: External force applied to the body
            external_torque: External torque applied to the body
            dt: Time step (if None, uses IMU sample rate)

        Returns:
            (accelerometer, gyroscope, magnetometer) readings
        """
        dt = dt or self.imu.dt

        # Update physics (simple integration)
        external_force = external_force or np.zeros(3)
        external_torque = external_torque or np.zeros(3)

        # Update linear motion (with gravity)
        gravity = np.array([0, 0, -9.81])  # m/s^2
        total_acceleration = external_force + gravity

        # Update velocity and position
        self.velocity += total_acceleration * dt
        self.position += self.velocity * dt

        # Update angular motion
        self.angular_velocity += external_torque * dt  # Simplified - no moment of inertia

        # Update orientation using quaternion integration
        self._integrate_orientation(dt)

        # Calculate true sensor values in sensor frame
        # Accelerometer measures linear acceleration + gravity in sensor frame
        true_accel_world = total_acceleration  # Linear acceleration in world frame
        R = self.imu._quaternion_to_rotation_matrix(self.orientation)
        true_accel_sensor = R.T @ true_accel_world

        # Gyroscope measures angular velocity
        true_gyro_sensor = self.angular_velocity

        # Get IMU readings
        accel, gyro, mag = self.imu.simulate_reading(
            true_accel_sensor, true_gyro_sensor, self.orientation, dt
        )

        self.time += dt

        return accel, gyro, mag

    def _integrate_orientation(self, dt: float):
        """Integrate orientation using quaternion mathematics"""
        # Convert angular velocity to quaternion derivative
        omega = self.angular_velocity
        omega_norm = np.linalg.norm(omega)

        if omega_norm > 1e-8:  # Avoid division by zero
            # Calculate quaternion derivative
            omega_quat = np.array([0, omega[0], omega[1], omega[2]])
            self.orientation += 0.5 * self._quat_multiply(omega_quat, self.orientation) * dt

            # Normalize quaternion
            self.orientation /= np.linalg.norm(self.orientation)

    def _quat_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])

# Example usage
if __name__ == "__main__":
    # Create IMU simulator
    imu_sim = IMUSimulator()

    # Simulate readings over time
    readings = []
    for i in range(1000):  # 10 seconds at 100Hz
        accel, gyro, mag = imu_sim.step()
        readings.append((accel, gyro, mag))

    print(f"IMU simulation completed. Generated {len(readings)} readings.")
    print(f"Sample accelerometer reading: {readings[0][0]}")
    print(f"Sample gyroscope reading: {readings[0][1]}")
    print(f"Sample magnetometer reading: {readings[0][2]}")
```

## Key Concepts Summary
- LiDAR simulation uses ray casting to detect obstacles with physics-based intersection
- Depth cameras require realistic noise modeling and distortion correction
- IMU sensors combine acceleration, rotation, and magnetic field measurements
- Proper sensor fusion combines multiple sensor inputs for improved accuracy
- Realistic noise models are essential for training robust perception systems

## References
- Gazebo Sensor Plugins Documentation
- Unity Sensor Simulation Resources
- "Probabilistic Robotics" by Sebastian Thrun
- "Robotics, Vision and Control" by Peter Corke
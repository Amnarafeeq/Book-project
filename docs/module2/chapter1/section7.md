---
title: Validation and Calibration
sidebar_position: 8
---

# Section 7: Validation and Calibration

## Learning Objectives
- Compare simulation results with real-world physics
- Calibrate simulation parameters for accuracy
- Implement validation metrics and benchmarks
- Troubleshoot common physics simulation issues

## Content Outline

### Simulation vs. Reality Comparison Methodologies
Validating physics simulations requires systematic comparison with real-world data:

#### Data Collection Strategies
- High-precision motion capture systems for ground truth
- Multiple sensor modalities for comprehensive validation
- Controlled experiments with known parameters
- Long-term data collection for statistical analysis

#### Comparison Metrics
- Position and orientation errors
- Velocity and acceleration differences
- Timing accuracy of events
- Energy conservation verification
- Statistical measures (RMSE, MAE, correlation coefficients)

#### Validation Protocols
- Repeatable experimental setups
- Standardized test scenarios
- Multiple trial runs for statistical significance
- Documentation of environmental conditions

### Parameter Calibration Techniques
Calibrating simulation parameters to match real-world behavior:

#### System Identification Methods
- Least squares optimization
- Maximum likelihood estimation
- Bayesian inference approaches
- Genetic algorithms for complex parameter spaces

#### Multi-Objective Optimization
- Balancing accuracy across different metrics
- Handling conflicting requirements
- Weighted cost functions
- Pareto-optimal solutions

#### Sensitivity Analysis
- Identifying most influential parameters
- Quantifying parameter uncertainty effects
- Robustness evaluation
- Parameter correlation analysis

### Benchmarking Tools and Metrics
Standardized tools and metrics for consistent validation:

#### Performance Metrics
- Real-time factor (RTF) measurement
- Frame rate stability
- Memory usage patterns
- Computational efficiency

#### Accuracy Metrics
- Absolute and relative position errors
- Orientation accuracy (in Euler angles or quaternions)
- Velocity and acceleration errors
- Collision detection accuracy

#### Reproducibility Metrics
- Deterministic behavior verification
- Cross-platform consistency
- Seed-based randomization validation

### Common Physics Simulation Problems and Solutions
Identifying and resolving typical issues in physics simulation:

#### Stability Issues
- Time step selection and integration methods
- Constraint violation problems
- Energy drift and numerical dissipation
- Contact stiffness and damping parameters

#### Accuracy Problems
- Mass and inertial property errors
- Friction and damping model mismatches
- Collision geometry approximations
- Sensor noise and bias modeling

#### Performance Bottlenecks
- Collision detection optimization
- Broad-phase algorithm selection
- Memory management for large scenes
- Parallel processing strategies

## Diagram/Code Notes
```
// Placeholder for validation script
// Comparison charts showing simulation vs. real data
// Calibration workflow diagram
```

## Implementation Example: Simulation Validation Framework

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from typing import Dict, List, Tuple, Optional
import json

class SimulationValidator:
    """Framework for validating physics simulations against real-world data"""

    def __init__(self):
        self.metrics = {}
        self.comparison_data = {}

    def add_reference_data(self, name: str, timestamps: np.ndarray,
                         positions: np.ndarray, orientations: np.ndarray = None):
        """Add reference (real-world) data for comparison"""
        self.comparison_data[name] = {
            'reference': {
                'timestamps': timestamps,
                'positions': positions,
                'orientations': orientations if orientations is not None else np.zeros_like(positions)
            }
        }

    def add_simulation_data(self, name: str, timestamps: np.ndarray,
                          positions: np.ndarray, orientations: np.ndarray = None):
        """Add simulation data for comparison"""
        if name not in self.comparison_data:
            self.comparison_data[name] = {}

        self.comparison_data[name]['simulation'] = {
            'timestamps': timestamps,
            'positions': positions,
            'orientations': orientations if orientations is not None else np.zeros_like(positions)
        }

    def calculate_position_error(self, ref_pos: np.ndarray, sim_pos: np.ndarray) -> Dict[str, float]:
        """Calculate position error metrics"""
        # Ensure arrays are the same length by interpolating if needed
        if len(ref_pos) != len(sim_pos):
            # Simple approach: take the shorter length
            min_len = min(len(ref_pos), len(sim_pos))
            ref_pos = ref_pos[:min_len]
            sim_pos = sim_pos[:min_len]

        errors = np.linalg.norm(ref_pos - sim_pos, axis=1)  # 3D distance errors

        metrics = {
            'rmse': np.sqrt(np.mean(errors**2)),
            'mae': np.mean(errors),
            'max_error': np.max(errors),
            'std_error': np.std(errors),
            'median_error': np.median(errors)
        }

        return metrics

    def calculate_orientation_error(self, ref_quat: np.ndarray, sim_quat: np.ndarray) -> Dict[str, float]:
        """Calculate orientation error metrics using quaternion difference"""
        if len(ref_quat) != len(sim_quat):
            min_len = min(len(ref_quat), len(sim_quat))
            ref_quat = ref_quat[:min_len]
            sim_quat = sim_quat[:min_len]

        # Calculate quaternion differences
        errors = []
        for i in range(len(ref_quat)):
            # Convert to rotation objects
            r_ref = R.from_quat(ref_quat[i])
            r_sim = R.from_quat(sim_quat[i])

            # Calculate relative rotation
            r_rel = r_ref.inv() * r_sim
            # Get rotation vector magnitude (angle in radians)
            angle = r_rel.magnitude()
            errors.append(angle)

        errors = np.array(errors)

        metrics = {
            'rmse_rad': np.sqrt(np.mean(errors**2)),
            'mae_rad': np.mean(errors),
            'max_error_rad': np.max(errors),
            'std_error_rad': np.std(errors),
            'median_error_rad': np.median(errors),
            'rmse_deg': np.sqrt(np.mean(errors**2)) * 180.0 / np.pi,
            'mae_deg': np.mean(errors) * 180.0 / np.pi,
            'max_error_deg': np.max(errors) * 180.0 / np.pi
        }

        return metrics

    def validate_trajectory(self, name: str) -> Dict[str, Dict[str, float]]:
        """Validate a complete trajectory"""
        if name not in self.comparison_data:
            raise ValueError(f"No data found for trajectory '{name}'")

        data = self.comparison_data[name]
        ref_data = data['reference']
        sim_data = data['simulation']

        results = {}

        # Calculate position errors
        pos_metrics = self.calculate_position_error(
            ref_data['positions'],
            sim_data['positions']
        )
        results['position'] = pos_metrics

        # Calculate orientation errors if available
        if ref_data['orientations'].size > 0 and sim_data['orientations'].size > 0:
            orient_metrics = self.calculate_orientation_error(
                ref_data['orientations'],
                sim_data['orientations']
            )
            results['orientation'] = orient_metrics

        # Store results
        self.metrics[name] = results

        return results

    def plot_comparison(self, name: str, save_path: str = None):
        """Plot comparison between reference and simulation data"""
        if name not in self.comparison_data:
            raise ValueError(f"No data found for trajectory '{name}'")

        data = self.comparison_data[name]
        ref_ts = data['reference']['timestamps']
        sim_ts = data['simulation']['timestamps']
        ref_pos = data['reference']['positions']
        sim_pos = data['simulation']['positions']

        # Create subplots for X, Y, Z positions
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))

        # Plot each axis
        for i, axis in enumerate(['X', 'Y', 'Z']):
            axes[i].plot(ref_ts, ref_pos[:, i], label='Reference', linewidth=2)
            axes[i].plot(sim_ts, sim_pos[:, i], label='Simulation', linewidth=2)
            axes[i].set_ylabel(f'Position {axis} (m)')
            axes[i].grid(True, alpha=0.3)
            axes[i].legend()

        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')

        plt.show()

    def generate_validation_report(self, name: str) -> str:
        """Generate a validation report"""
        if name not in self.metrics:
            self.validate_trajectory(name)

        metrics = self.metrics[name]

        report = f"Validation Report for '{name}'\n"
        report += "=" * 50 + "\n\n"

        # Position metrics
        if 'position' in metrics:
            pos_metrics = metrics['position']
            report += "Position Metrics:\n"
            report += f"  RMSE: {pos_metrics['rmse']:.4f} m\n"
            report += f"  MAE:  {pos_metrics['mae']:.4f} m\n"
            report += f"  Max Error: {pos_metrics['max_error']:.4f} m\n"
            report += f"  Std Dev:   {pos_metrics['std_error']:.4f} m\n\n"

        # Orientation metrics
        if 'orientation' in metrics:
            orient_metrics = metrics['orientation']
            report += "Orientation Metrics:\n"
            report += f"  RMSE: {orient_metrics['rmse_deg']:.4f} deg\n"
            report += f"  MAE:  {orient_metrics['mae_deg']:.4f} deg\n"
            report += f"  Max Error: {orient_metrics['max_error_deg']:.4f} deg\n\n"

        return report

# Example usage
if __name__ == "__main__":
    # Create sample reference and simulation data
    t = np.linspace(0, 10, 1000)  # 10 seconds, 1000 samples

    # Reference trajectory (real-world data)
    ref_positions = np.column_stack([
        2 * np.sin(0.5 * t),           # X: sinusoidal motion
        1.5 * np.cos(0.3 * t),         # Y: different frequency
        -0.1 * t + 0.1 * t**2         # Z: parabolic (simulating gravity effect)
    ])

    # Simulation trajectory (with some errors)
    sim_positions = ref_positions + np.random.normal(0, 0.01, ref_positions.shape)  # Add noise
    sim_positions[:, 2] *= 0.95  # Systematic error in Z direction

    # Create validator
    validator = SimulationValidator()

    # Add data
    validator.add_reference_data('test_trajectory', t, ref_positions)
    validator.add_simulation_data('test_trajectory', t, sim_positions)

    # Validate
    results = validator.validate_trajectory('test_trajectory')

    # Print results
    print("Validation Results:")
    for metric_type, values in results.items():
        print(f"{metric_type.upper()} Metrics:")
        for key, value in values.items():
            print(f"  {key}: {value:.6f}")

    # Generate report
    report = validator.generate_validation_report('test_trajectory')
    print("\n" + report)
```

## Implementation Example: Parameter Calibration Tool

```python
import numpy as np
from scipy.optimize import minimize
from typing import Callable, Dict, List, Tuple
import matplotlib.pyplot as plt

class PhysicsCalibrator:
    """Tool for calibrating physics simulation parameters"""

    def __init__(self, simulation_function: Callable,
                 reference_data: Dict[str, np.ndarray]):
        """
        Initialize calibrator

        Args:
            simulation_function: Function that takes parameters and returns simulation results
            reference_data: Dictionary containing reference measurements
        """
        self.sim_func = simulation_function
        self.ref_data = reference_data
        self.optimization_history = []

    def cost_function(self, params: np.ndarray,
                     param_names: List[str],
                     weights: Dict[str, float] = None) -> float:
        """
        Calculate cost function for optimization

        Args:
            params: Parameter values to test
            param_names: Names of parameters corresponding to values
            weights: Weights for different metrics

        Returns:
            Cost value (lower is better)
        """
        # Create parameter dictionary
        param_dict = {name: val for name, val in zip(param_names, params)}

        # Run simulation with these parameters
        sim_results = self.sim_func(param_dict)

        # Calculate errors for different metrics
        total_cost = 0.0

        for key in self.ref_data.keys():
            if key in sim_results:
                ref = self.ref_data[key]
                sim = sim_results[key]

                # Ensure same length
                min_len = min(len(ref), len(sim))
                ref = ref[:min_len]
                sim = sim[:min_len]

                # Calculate error (RMSE)
                error = np.sqrt(np.mean((ref - sim)**2))

                # Apply weight if specified
                weight = weights.get(key, 1.0) if weights else 1.0
                total_cost += weight * error

        # Store in history for analysis
        self.optimization_history.append({
            'params': param_dict.copy(),
            'cost': total_cost
        })

        return total_cost

    def calibrate(self,
                 initial_params: Dict[str, float],
                 bounds: Dict[str, Tuple[float, float]],
                 weights: Dict[str, float] = None,
                 method: str = 'L-BFGS-B') -> Dict[str, float]:
        """
        Calibrate parameters using optimization

        Args:
            initial_params: Initial parameter guesses
            bounds: Parameter bounds as (min, max) tuples
            weights: Weights for different metrics
            method: Optimization method

        Returns:
            Optimized parameters
        """
        # Prepare for optimization
        param_names = list(initial_params.keys())
        initial_values = list(initial_params.values())

        # Set up bounds
        param_bounds = [bounds[name] for name in param_names]

        # Optimize
        result = minimize(
            fun=lambda params: self.cost_function(params, param_names, weights),
            x0=initial_values,
            method=method,
            bounds=param_bounds,
            options={'disp': True}
        )

        # Return optimized parameters
        optimized_params = {name: val for name, val in zip(param_names, result.x)}

        return optimized_params

    def sensitivity_analysis(self,
                           base_params: Dict[str, float],
                           param_ranges: Dict[str, Tuple[float, float]],
                           steps: int = 10) -> Dict[str, np.ndarray]:
        """
        Perform sensitivity analysis for parameters

        Args:
            base_params: Base parameter values
            param_ranges: Ranges to test for each parameter
            steps: Number of steps for each parameter

        Returns:
            Sensitivity results for each parameter
        """
        results = {}

        for param_name, (min_val, max_val) in param_ranges.items():
            costs = []
            param_values = np.linspace(min_val, max_val, steps)

            for val in param_values:
                test_params = base_params.copy()
                test_params[param_name] = val

                # Run simulation with modified parameter
                sim_results = self.sim_func(test_params)

                # Calculate cost
                cost = 0.0
                for key in self.ref_data.keys():
                    if key in sim_results:
                        ref = self.ref_data[key]
                        sim = sim_results[key]

                        min_len = min(len(ref), len(sim))
                        ref = ref[:min_len]
                        sim = sim[:min_len]

                        cost += np.sqrt(np.mean((ref - sim)**2))

                costs.append(cost)

            results[param_name] = np.column_stack([param_values, costs])

        return results

    def plot_sensitivity(self, sensitivity_results: Dict[str, np.ndarray]):
        """Plot sensitivity analysis results"""
        n_params = len(sensitivity_results)
        fig, axes = plt.subplots(n_params, 1, figsize=(10, 4*n_params))
        if n_params == 1:
            axes = [axes]

        for i, (param_name, data) in enumerate(sensitivity_results.items()):
            param_values = data[:, 0]
            costs = data[:, 1]

            axes[i].plot(param_values, costs, 'b-', linewidth=2)
            axes[i].set_xlabel(f'{param_name}')
            axes[i].set_ylabel('Cost')
            axes[i].grid(True, alpha=0.3)
            axes[i].set_title(f'Sensitivity: {param_name}')

        plt.tight_layout()
        plt.show()

# Example simulation function for calibration
def example_simulation(params: Dict[str, float]) -> Dict[str, np.ndarray]:
    """
    Example simulation: simple mass-spring-damper system
    """
    # Extract parameters
    mass = params.get('mass', 1.0)
    damping = params.get('damping', 0.1)
    spring_const = params.get('spring_const', 10.0)
    initial_pos = params.get('initial_pos', 1.0)

    # Time vector
    t = np.linspace(0, 5, 500)

    # Simulate mass-spring-damper system
    # Response: x(t) = A * exp(-zeta*wn*t) * cos(wd*t + phi)
    wn = np.sqrt(spring_const / mass)  # Natural frequency
    zeta = damping / (2 * np.sqrt(mass * spring_const))  # Damping ratio
    wd = wn * np.sqrt(1 - zeta**2) if zeta < 1 else 0  # Damped frequency

    if zeta < 1:  # Underdamped
        A = initial_pos
        phi = 0
        position = A * np.exp(-zeta * wn * t) * np.cos(wd * t + phi)
    else:  # Critically or overdamped - simplified
        position = initial_pos * np.exp(-damping * t)

    # Calculate velocity (derivative of position)
    velocity = np.gradient(position, t[1] - t[0])

    return {
        'position': position,
        'velocity': velocity,
        'time': t
    }

# Example usage
if __name__ == "__main__":
    # Generate reference data (real system with known parameters)
    true_params = {'mass': 1.2, 'damping': 0.15, 'spring_const': 9.5, 'initial_pos': 1.0}
    ref_data = example_simulation(true_params)

    # Add some noise to make it more realistic
    ref_data['position'] += np.random.normal(0, 0.01, len(ref_data['position']))

    # Initial guess for parameters
    initial_guess = {'mass': 1.0, 'damping': 0.1, 'spring_const': 10.0, 'initial_pos': 1.0}

    # Parameter bounds
    bounds = {
        'mass': (0.5, 2.0),
        'damping': (0.01, 0.5),
        'spring_const': (5.0, 15.0),
        'initial_pos': (0.5, 1.5)
    }

    # Create calibrator
    calibrator = PhysicsCalibrator(example_simulation,
                                 {'position': ref_data['position']})

    # Calibrate parameters
    print("Starting calibration...")
    optimized_params = calibrator.calibrate(initial_guess, bounds)

    print("Optimized parameters:")
    for param, value in optimized_params.items():
        true_value = true_params[param]
        print(f"  {param}: {value:.4f} (true: {true_value:.4f}, error: {abs(value-true_value)/true_value*100:.2f}%)")

    # Perform sensitivity analysis
    param_ranges = {
        'mass': (0.8, 1.5),
        'damping': (0.1, 0.2),
        'spring_const': (8.0, 11.0)
    }

    sensitivity = calibrator.sensitivity_analysis(optimized_params, param_ranges)

    # Plot sensitivity
    calibrator.plot_sensitivity(sensitivity)
```

## Implementation Example: Troubleshooting and Diagnostics

```python
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple
import time

class PhysicsDiagnostics:
    """Tools for diagnosing physics simulation issues"""

    def __init__(self):
        self.performance_log = []
        self.stability_metrics = []
        self.warning_log = []

    def check_time_step_stability(self, time_step: float,
                                max_recommended: float = 0.01) -> Dict[str, any]:
        """Check if time step is appropriate for stability"""
        issues = []

        if time_step > max_recommended:
            issues.append(f"Time step ({time_step}s) may be too large, recommend < {max_recommended}s")

        # Check for consistency
        if hasattr(self, 'prev_time_step') and abs(time_step - self.prev_time_step) > 0.001:
            issues.append("Large variation in time step detected")

        self.prev_time_step = time_step

        return {
            'is_stable': len(issues) == 0,
            'issues': issues,
            'recommendation': f"Use time step < {max_recommended}s for better stability" if issues else "Time step appears appropriate"
        }

    def check_energy_conservation(self, kinetic_energy: float,
                                potential_energy: float,
                                initial_total_energy: float,
                                tolerance: float = 0.1) -> Dict[str, any]:
        """Check if energy is being conserved (for conservative systems)"""
        current_total = kinetic_energy + potential_energy
        energy_error = abs(current_total - initial_total_energy) / abs(initial_total_energy)

        issues = []
        if energy_error > tolerance:
            issues.append(f"Energy not conserved: {energy_error*100:.2f}% error")

        return {
            'energy_error_percent': energy_error * 100,
            'is_conserved': energy_error <= tolerance,
            'issues': issues,
            'current_total': current_total,
            'initial_total': initial_total_energy
        }

    def check_constraint_violations(self, constraints: List[Dict]) -> Dict[str, any]:
        """Check for constraint violations"""
        violations = []
        max_violation = 0.0

        for i, constraint in enumerate(constraints):
            if 'error' in constraint:
                error = abs(constraint['error'])
                if error > constraint.get('tolerance', 0.001):
                    violations.append({
                        'constraint_id': i,
                        'error': error,
                        'tolerance': constraint.get('tolerance', 0.001),
                        'type': constraint.get('type', 'unknown')
                    })
                max_violation = max(max_violation, error)

        return {
            'violations': violations,
            'max_violation': max_violation,
            'total_violations': len(violations),
            'is_valid': len(violations) == 0
        }

    def check_collision_geometry(self, objects: List[Dict]) -> Dict[str, any]:
        """Check collision geometry for common issues"""
        issues = []

        for i, obj in enumerate(objects):
            if 'collision_mesh' in obj:
                mesh = obj['collision_mesh']

                # Check for degenerate geometry
                if mesh.get('volume', 0) <= 0:
                    issues.append(f"Object {i}: Collision mesh has zero or negative volume")

                # Check for extremely small or large scales
                if mesh.get('min_dimension', float('inf')) < 0.001:
                    issues.append(f"Object {i}: Very small collision geometry (< 1mm)")

                if mesh.get('max_dimension', 0) > 1000:
                    issues.append(f"Object {i}: Very large collision geometry (> 1000m)")

            # Check mass properties
            mass = obj.get('mass', 1.0)
            if mass <= 0:
                issues.append(f"Object {i}: Invalid mass ({mass})")

            # Check for extremely light or heavy objects
            if mass < 0.001:
                issues.append(f"Object {i}: Very light object ({mass}kg) - may cause instability")

            if mass > 10000:
                issues.append(f"Object {i}: Very heavy object ({mass}kg) - may cause instability")

        return {
            'issues': issues,
            'object_count': len(objects),
            'has_issues': len(issues) > 0
        }

    def performance_monitor(self, frame_time: float,
                          expected_frame_time: float = 0.016) -> Dict[str, any]:
        """Monitor performance and detect bottlenecks"""
        rt_factor = expected_frame_time / frame_time if frame_time > 0 else float('inf')

        issues = []
        if frame_time > expected_frame_time * 2:  # More than 2x expected time
            issues.append(f"Performance bottleneck detected: {frame_time*1000:.1f}ms per frame")

        self.performance_log.append({
            'timestamp': time.time(),
            'frame_time': frame_time,
            'rt_factor': rt_factor,
            'issues': issues.copy()
        })

        return {
            'frame_time_ms': frame_time * 1000,
            'real_time_factor': rt_factor,
            'issues': issues,
            'is_performing_well': len(issues) == 0
        }

    def generate_diagnostics_report(self) -> str:
        """Generate a comprehensive diagnostics report"""
        report = "Physics Simulation Diagnostics Report\n"
        report += "=" * 50 + "\n\n"

        # Performance summary
        if self.performance_log:
            avg_frame_time = np.mean([log['frame_time'] for log in self.performance_log])
            avg_rt_factor = np.mean([log['rt_factor'] for log in self.performance_log if log['rt_factor'] != float('inf')])

            report += f"Performance Summary:\n"
            report += f"  Average frame time: {avg_frame_time*1000:.2f} ms\n"
            report += f"  Average RT factor: {avg_rt_factor:.2f}\n\n"

        # Warning summary
        if self.warning_log:
            report += f"Warnings encountered: {len(self.warning_log)}\n"
            for i, warning in enumerate(self.warning_log[:5]):  # Show first 5
                report += f"  {i+1}. {warning}\n"
            if len(self.warning_log) > 5:
                report += f"  ... and {len(self.warning_log) - 5} more\n\n"
        else:
            report += "No warnings detected.\n\n"

        return report

# Example usage
if __name__ == "__main__":
    # Create diagnostics instance
    diag = PhysicsDiagnostics()

    # Simulate checking various aspects
    time_step_check = diag.check_time_step_stability(0.02)  # Potentially too large
    print("Time Step Check:")
    print(f"  Stable: {time_step_check['is_stable']}")
    print(f"  Issues: {time_step_check['issues']}")
    print()

    energy_check = diag.check_energy_conservation(10.5, 5.2, 15.0, tolerance=0.05)
    print("Energy Conservation Check:")
    print(f"  Conserved: {energy_check['is_conserved']}")
    print(f"  Error: {energy_check['energy_error_percent']:.2f}%")
    print(f"  Issues: {energy_check['issues']}")
    print()

    constraints = [
        {'error': 0.002, 'tolerance': 0.001, 'type': 'joint'},
        {'error': 0.0005, 'tolerance': 0.001, 'type': 'contact'}
    ]
    constraint_check = diag.check_constraint_violations(constraints)
    print("Constraint Violation Check:")
    print(f"  Valid: {constraint_check['is_valid']}")
    print(f"  Violations: {constraint_check['total_violations']}")
    print(f"  Max violation: {constraint_check['max_violation']:.6f}")
    print()

    objects = [
        {'mass': 0.0001, 'collision_mesh': {'min_dimension': 0.0005, 'volume': 0.001}},
        {'mass': 5.0, 'collision_mesh': {'min_dimension': 0.1, 'volume': 0.1}}
    ]
    geometry_check = diag.check_collision_geometry(objects)
    print("Collision Geometry Check:")
    print(f"  Has issues: {geometry_check['has_issues']}")
    print(f"  Issues: {geometry_check['issues']}")
    print()

    performance_check = diag.performance_monitor(0.05)  # Slow frame
    print("Performance Check:")
    print(f"  Frame time: {performance_check['frame_time_ms']:.1f} ms")
    print(f"  RT factor: {performance_check['real_time_factor']:.2f}")
    print(f"  Issues: {performance_check['issues']}")
    print()

    # Generate report
    report = diag.generate_diagnostics_report()
    print(report)
```

## Key Concepts Summary
- Validation requires systematic comparison between simulation and real-world data
- Calibration techniques optimize parameters to match reference behavior
- Proper metrics and benchmarks ensure consistent evaluation
- Diagnostics help identify and resolve common physics simulation issues
- Sensitivity analysis reveals parameter importance and robustness

## References
- "Verification and Validation in Scientific Computing" by William Oberkampf
- Gazebo Simulation Documentation
- Physics Simulation Best Practices
- "Computer Graphics: Principles and Practice" for validation methodologies
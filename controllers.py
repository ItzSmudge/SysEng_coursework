from abc import ABC, abstractmethod
import numpy as np
import control as ct 
class Controller(ABC):
    @abstractmethod
    def get_action(self, state, target_pos=0.0):
        pass

    @abstractmethod
    def reset(self):
        pass

class PIDController(Controller):

    def __init__(self, kp_theta=0.0, kd_theta=0.0, ki_theta=0.0, kp_x=0.0, kd_x=0.0, ki_x=0.0, dt=0.001, i_limit_theta=10.0, i_limit_x=10.0):
        self.kp_theta = kp_theta
        self.ki_theta = ki_theta
        self.kd_theta = kd_theta

        self.kp_x = kp_x
        self.ki_x = ki_x
        self.kd_x = kd_x

        self.dt = dt 

        self.integral_theta = 0.0 
        self.integral_x = 0.0 
        # self.prev_error_theta = 0.0 
        # self.prev_error_x = 0.0

        self.i_limit_theta = i_limit_theta
        self.i_limit_x = i_limit_x
    
    def get_action(self, state, target_pos=[0.0, 0.0]):
        x, x_dot, theta, theta_dot = state
        
        target_x = target_pos[0]

        error_x = target_x - x 
        self.integral_x += error_x*self.dt 
        self.integral_x = np.clip(self.integral_x, -self.i_limit_x, self.i_limit_x)

        desired_theta = (self.kp_x*error_x + self.ki_x*self.integral_x - self.kd_x*x_dot)

        error_theta = desired_theta - theta
        self.integral_theta += error_theta*self.dt
        self.integral_theta = np.clip(self.integral_theta, -self.i_limit_theta, self.i_limit_theta)

        force = (self.kp_theta*error_theta + self.ki_theta*self.integral_theta - self.kd_theta*theta_dot)

        return force


    def reset(self):
        self.integral_theta = 0.0 
        self.integral_x = 0.0
        # self.prev_error_theta = 0.0 
        # self.prev_error_x = 0.0

    def set_kvalues(self, kp_theta, kd_theta, ki_theta, kp_x, kd_x, ki_x):
        self.kp_theta = kp_theta
        self.ki_theta = ki_theta
        self.kd_theta = kd_theta

        self.kp_x = kp_x
        self.ki_x = ki_x
        self.kd_x = kd_x

class LQRController(Controller):
    def __init__(self, M, m, l, b, I=None, g=9.81, Q=None, R=None):
        if I is None:
            self.I = (1/3) * m * (l**2)
        else:
            self.I = self.I
    
        
        # linearise
        denominator = self.I*(M + m) + M*m*(l**2)

        # MatA jacobian with respect to state [x, xdot, theta, theta dot]
        
        A = np.array([
            [0, 1, 0, 0],
            [0, -(self.I + m*l**2)*b / denominator,  (m**2 * l**2 * g) / denominator, 0],
            [0, 0, 0, 1],
            [0, -(m*l*b) / denominator,(m * l * g * (M + m)) / denominator, 0]
        ])

        # Matrix B (Jacobian with respect to input u)
        B = np.array([
            [0],
            [(self.I + m*l**2) / denominator],
            [0],
            [(m*l) / denominator]
        ])

        # Default Weights if none provided
        if Q is None:
            # Penalities: [Pos, Vel, Angle, AngVel]
            Q = np.diag([1.0, 1.0, 10.0, 1.0]) 
        if R is None:
            R = np.array([[0.1]]) # Penalty on motor force

        self.Q = Q
        self.R = R
        self.M = M
        self.m = m
        self.l = l
        self.b = b
        self.g = g

        # K, S, E = control.lqr(A, B, Q, R)
        # control.lqr returns K such that u = -Kx
        self.K, _, _ = ct.lqr(A, B, self.Q, self.R)
        
        print("lqr gain matrix :", self.K)

    def get_action(self, state, target_pos=[0.0, 0.0]):
        # state [x, x_dot, theta, theta_dot]
        # target[target_x, 0, 0, 0]
        
        target_x = target_pos[0] if isinstance(target_pos, (list, tuple, np.ndarray)) else target_pos
        
        current_state = np.array(state)
        desired_state = np.array([target_x, 0.0, 0.0, 0.0])
        error = current_state - desired_state
        force = -np.dot(self.K, error)

        return float(force[0])

    def reset(self):
        pass


class TrajectoryPIDController(Controller):
    """
    PID controller with trajectory planning for point-to-point movement.
    Generates smooth reference trajectories to move from start to target position.
    """
    
    def __init__(self, kp_theta=75.0, kd_theta=2.0, ki_theta=0.0, 
                 kp_x=5.0, kd_x=8.0, ki_x=0.1, dt=0.001, 
                 trajectory_duration=5.0):
        self.kp_theta = kp_theta
        self.ki_theta = ki_theta
        self.kd_theta = kd_theta

        self.kp_x = kp_x
        self.ki_x = ki_x
        self.kd_x = kd_x

        self.dt = dt 
        self.trajectory_duration = trajectory_duration

        self.integral_theta = 0.0 
        self.integral_x = 0.0
        
        # Trajectory state
        self.t_start = None
        self.x_start = None
        self.x_target = None
        self.trajectory_active = False
    
    def generate_trajectory(self, t_elapsed):
        """
        Generate smooth S-curve trajectory using cubic polynomial.
        Returns: (position_reference, velocity_reference)
        """
        if not self.trajectory_active or t_elapsed >= self.trajectory_duration:
            # Trajectory complete - hold at target
            return self.x_target, 0.0
        
        # Normalized time [0, 1]
        s = t_elapsed / self.trajectory_duration
        
        # S-curve: smooth acceleration and deceleration
        # Position: cubic interpolation
        x_ref = self.x_start + (self.x_target - self.x_start) * (3*s**2 - 2*s**3)
        
        # Velocity: derivative of position
        x_dot_ref = (self.x_target - self.x_start) * (6*s - 6*s**2) / self.trajectory_duration
        
        return x_ref, x_dot_ref
    
    def start_trajectory(self, current_x, target_x):
        """Initialize a new trajectory from current position to target"""
        self.x_start = current_x
        self.x_target = target_x
        self.t_start = 0
        self.trajectory_active = True
        print(f"Starting trajectory: {current_x:.3f}m → {target_x:.3f}m over {self.trajectory_duration:.1f}s")
    
    def get_action(self, state, target_pos=[0.0, 0.0], current_time=None):
        x, x_dot, theta, theta_dot = state

        if abs(x - target_pos[0]) < 0.1:
            return 0.0
        
        # Initialize trajectory on first call or if target changed
        if not self.trajectory_active or self.x_target != target_pos[0]:
            self.start_trajectory(x, target_pos[0])
        
        # Update trajectory time
        if self.t_start is not None:
            self.t_start += self.dt
        
        # Get reference trajectory
        x_ref, x_dot_ref = self.generate_trajectory(self.t_start)
        
        # Position control with feedforward
        error_x = x_ref - x
        error_x_dot = x_dot_ref - x_dot
        self.integral_x += error_x * self.dt
        
        # Anti-windup: limit integral
        max_integral = 10.0
        self.integral_x = np.clip(self.integral_x, -max_integral, max_integral)
        
        # Position PID → desired angle
        desired_theta = (self.kp_x * error_x) + (self.kd_x * error_x_dot) + (self.ki_x * self.integral_x)
        desired_theta *= -1.0

        # Limit desired angle to prevent extreme tilts
        max_desired_theta = 0.3  # ~17 degrees max lean
        desired_theta = np.clip(desired_theta, -max_desired_theta, max_desired_theta)
        
        # Angle control → force to achieve desired angle
        error_theta = desired_theta - theta
        self.integral_theta += error_theta * self.dt
        
        # Anti-windup for angle
        self.integral_theta = np.clip(self.integral_theta, -max_integral, max_integral)
        
        derivative_theta = 0 - theta_dot
        
        force_out = (self.kp_theta * error_theta) + (self.ki_theta * self.integral_theta) + (self.kd_theta * derivative_theta)
        
        return force_out

    def reset(self):
        self.integral_theta = 0.0 
        self.integral_x = 0.0
        self.t_start = None
        self.x_start = None
        self.x_target = None
        self.trajectory_active = False

    def set_kvalues(self, kp_theta, kd_theta, ki_theta, kp_x, kd_x, ki_x, trajectory_duration=None):
        self.kp_theta = kp_theta
        self.ki_theta = ki_theta
        self.kd_theta = kd_theta

        self.kp_x = kp_x
        self.ki_x = ki_x
        self.kd_x = kd_x
        
        if trajectory_duration is not None:
            self.trajectory_duration = trajectory_duration


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

    def __init__(self, kp_theta=0.0, kd_theta=0.0, ki_theta=0.0, kp_x=0.0, kd_x=0.0, ki_x=0.0, dt=0.001):
        self.kp_theta = kp_theta
        self.ki_theta = ki_theta
        self.kd_theta = kd_theta

        self.kp_x = kp_x
        self.ki_x = ki_x
        self.kd_x = kd_x

        self.dt = dt 

        self.integral_theta = 0.0 
        self.integral_x = 0.0 
        self.prev_error_theta = 0.0 
        self.prev_error_x = 0.0
    
    def get_action(self, state, target_pos=[0.0, 0.0]):
        x, x_dot, theta, theta_dot = state
        
        error_x = target_pos[0] - x 
        self.integral_x += error_x * self.dt  # Assuming dt=0.001 for integral calculation
        derivative_x = 0 - x_dot  #d/dt(target - x) = 0 - velocity

        desired_theta = (self.kp_x * error_x) + (self.ki_x * self.integral_x) + (self.kd_x * derivative_x)

        error_theta = desired_theta - theta
        self.integral_theta += error_theta * self.dt
        derivative_theta = 0 - theta_dot 

        theta_force = (self.kp_theta * error_theta) + (self.ki_theta * self.integral_theta) + (self.kd_theta * derivative_theta)
        x_force = (self.kp_x * error_x) + (self.ki_x * self.integral_x) + (self.kd_x * derivative_x)

        force_out = theta_force + x_force

        return force_out


    def reset(self):
        self.integral_theta = 0.0 
        self.integral_x = 0.0
        self.prev_error_theta = 0.0 
        self.prev_error_x = 0.0

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

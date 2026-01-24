from abc import ABC, abstractmethod
import numpy as np

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
        
        # Position control -> desired position
        error_x = target_pos[0] - x 
        self.integral_x += error_x * self.dt
        derivative_x = (0 - x_dot ) / self.dt
        
        force_x = (self.kp_x * error_x) + (self.ki_x * self.integral_x) + (self.kd_x * derivative_x)
        
        # Angle control -> force to achieve desired angle
        error_theta = target_pos[1] - theta
        self.integral_theta += error_theta * self.dt
        derivative_theta = (0 - theta_dot) / self.dt
        
        force_theta = (self.kp_theta * error_theta) + (self.ki_theta * self.integral_theta) + (self.kd_theta * derivative_theta)
        
        return force_x + force_theta


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

    def __init__(self):
        pass

    def get_action(self):
        pass

    def reset(self):
        pass
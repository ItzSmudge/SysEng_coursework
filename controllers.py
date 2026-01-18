from abc import ABC, abstractmethod
import numpy as np

class Controller(ABC):
    @abstractmethod
    def get_action(self, state, target_pos=0.0):
        pass

    @abstractmethod
    def reset(self):
        pass

#test commit
class PIDController(Controller):

    def __init__(self, kp_theta=0.0, kd_theta=0.0, ki_theta=0.0, kp_x=0.0, kd_x=0.0, ki_x=0.0):
        self.kp_theta = kp_theta
        self.ki_theta = ki_theta
        self.kd_theta = kd_theta

        self.kp_x = kp_x
        self.ki_x = ki_x
        self.kd_x = kd_x

        self.integral_theta = 0.0 
        self.integral_x = 0.0 
        self.prev_error_theta = 0.0 
        self.prev_error_x = 0.0
    
    def get_action(self, state, target_pos=0.0):
        x, x_dot, theta, theta_dot = state
        
        error_x = target_pos - x
        self.integral_x += error_x 
        derivative_x = 0 - x_dot  #d/dt(target - x) = 0 - velocity

        desired_theta = (self.kp_x * error_x) + (self.ki_x * self.integral_x) + (self.kd_x * derivative_x)

        error_theta = desired_theta - theta
        self.integral_theta += error_theta
        derivative_theta = 0 - theta_dot 

        force_out = (self.kp_theta*error_theta) + (self.ki_theta*self.integral_theta) + (self.kd_theta*derivative_theta)

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

    def __init__(self):
        pass

    def get_action(self):
        pass

    def reset(self):
        pass
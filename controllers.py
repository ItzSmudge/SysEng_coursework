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

    def __init__(self, kp_theta=0.0, kd_theta=0.0, ki_theta=0.0, kp_x=0.0, kd_x=0.0, ki_x=0.0, dt=0.001, i_limit_theta=10.0, i_limit_x=10.0, window_size=10,filter_enabled=False):
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
        self.filter_enabled = filter_enabled
        if self.filter_enabled:
            self.filter_theta = MovingAverageFilter(window_size)
            self.filter_x = MovingAverageFilter(window_size)
        else:
            self.filter_theta = None
            self.filter_x = None

    def get_action(self, state, target_pos=[0.0, 0.0]):
        x, x_dot, theta, theta_dot = state

        if self.filter_enabled:
            theta = self.filter_theta.apply(theta)
            x = self.filter_x.apply(x)
        
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

    def __init__(self):
        pass

    def get_action(self):
        pass

    def reset(self):
        pass
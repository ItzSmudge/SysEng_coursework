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
    def __init__(self, kp_theta=0.0, kd_theta=0.0, ki_theta=0.0, 
                 kp_x=0.0, kd_x=0.0, ki_x=0.0, dt=0.01):
    
        self.kp_theta = kp_theta
        self.ki_theta = ki_theta
        self.kd_theta = kd_theta

        
        self.kp_x = kp_x
        self.ki_x = ki_x
        self.kd_x = kd_x

    
        self.dt = dt

        self.integral_theta = 0.0 
        self.integral_x = 0.0 
        
        # limits - this was claude
        self.integral_theta_max = 50.0
        self.integral_x_max = 50.0
    
    def get_action(self, state, target_pos=0.0):
        x, x_dot, theta, theta_dot = state
      
        error_x = target_pos - x 
        
        #  claude suggestion
        self.integral_x += error_x * self.dt
        self.integral_x = np.clip(self.integral_x, 
                                  -self.integral_x_max, 
                                  self.integral_x_max)
        
        # Derivative (rate of change of error ≈ -velocity for constant target)
        derivative_x = -x_dot

        # Compute desired pendulum angle
        desired_theta = (self.kp_x * error_x + 
                        self.ki_x * self.integral_x + 
                        self.kd_x * derivative_x)


        error_theta = desired_theta - theta
        
        # more anti windup code
        self.integral_theta += error_theta * self.dt
        self.integral_theta = np.clip(self.integral_theta,
                                      -self.integral_theta_max,
                                      self.integral_theta_max)
        
        derivative_theta = -theta_dot

        # Compute control force
        force_out = (self.kp_theta * error_theta + 
                    self.ki_theta * self.integral_theta + 
                    self.kd_theta * derivative_theta)

        return force_out

    def reset(self):
        self.integral_theta = 0.0 
        self.integral_x = 0.0

    def set_kvalues(self, kp_theta, kd_theta, ki_theta, kp_x, kd_x, ki_x):
        self.kp_theta = kp_theta
        self.ki_theta = ki_theta
        self.kd_theta = kd_theta

        self.kp_x = kp_x
        self.ki_x = ki_x
        self.kd_x = kd_x
    
    def set_dt(self, dt):
        self.dt = dt
    
    def set_integral_limits(self, theta_max=50.0, x_max=50.0):
        #update anti windup limits
        self.integral_theta_max = theta_max
        self.integral_x_max = x_max


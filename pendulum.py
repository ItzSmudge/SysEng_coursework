import numpy as np
import scipy.integrate


class Pendulum:

    def __init__(self, M, m, l, b, dt=0.001, I=None, g=9.81, controller=None):
        self.I = I if I is not None else (1/3) *m*(l**2)
        self.M = M
        self.m = m
        self.l = l
        self.b = b
        self.g = g
        self.denominator = self.I*(self.M + self.m) + self.M*self.m*(self.l**2)
        self.dt = dt

    def dynamics(self, t, state, u):
        x = state[0]
        x_dot = state[1]
        theta = state[2]
        theta_dot = state[3]



        next_state = np.zeros(4)
        next_state[0] = x_dot
        next_state[1] =  (-(self.I + self.m*self.l**2)*self.b*x_dot/self.denominator) + theta * (self.m**2 * self.l**2 * self.g / self.denominator) + u * (self.I + self.m*self.l**2)/self.denominator
        next_state[2] = theta_dot
        next_state[3] =  (-(self.m*self.l*self.b)*x_dot/self.denominator) + theta * (self.m*self.l*self.g*(self.M + self.m)/self.denominator) + u * (self.m*self.l)/self.denominator

        return next_state

    def step(self, state, u, dt=None):
        if dt is None:
            dt = self.dt

        # return scipy.integrate.solve_ivp(lambda t, y: self.dynamics(t, y, u),
        #                              [0, dt], state, method='RK45').y[:, -1]
        k1 = self.dynamics(0, state, u)
        k2 = self.dynamics(0, state + 0.5 * dt * k1, u)
        k3 = self.dynamics(0, state + 0.5 * dt * k2, u)         
        k4 = self.dynamics(0, state + dt * k3, u)
        return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def simulate(self, state, controller, steps, target_pos=0.0):
        trajectory = np.zeros((steps, len(state)))
        trajectory[0] = state
        for t in range(1, steps):
            u = controller.get_action(state, target_pos)
            state = self.step(state, u)
            trajectory[t] = state
        return trajectory
    
    
        

import numpy as np
import scipy.integrate


class Pendulum:

    def __init__(self, M, m, l, b, dt=0.001, I=None, g=9.81, controller=None, mode="1", disturbance_level=1, noise_std_dev=0.01):
        self.I = I if I is not None else (1/3) *m*(l**2)
        self.M = M
        self.m = m
        self.l = l
        self.b = b
        self.g = g
        self.denominator = self.I*(self.M + self.m) + self.M*self.m*(self.l**2)
        self.dt = dt
        self.mode = mode
        self.disturbance_level = disturbance_level
        self.apply_disturbance = False
        self.noise_std_dev = noise_std_dev

    def dynamics(self, t, state, u, Fd=0.0):
        x = state[0]
        x_dot = state[1]
        theta = state[2]
        theta_dot = state[3]

        next_state = np.zeros(4)
        next_state[0] = x_dot
        next_state[1] =  (-(self.I + self.m*self.l**2)*self.b*x_dot/self.denominator) + theta * (self.m**2 * self.l**2 * self.g / self.denominator) + u * (self.I + self.m*self.l**2)/self.denominator
        next_state[2] = theta_dot
        next_state[3] =  (-(self.m*self.l*self.b)*x_dot/self.denominator) + theta * (self.m*self.l*self.g*(self.M + self.m)/self.denominator) + u * (self.m*self.l)/self.denominator

        next_state[1] += Fd[0,0] * (self.I + self.m*self.l**2) / self.denominator
        next_state[3] += Fd[0,1] * (self.m*self.l) / self.denominator

        return next_state

    def step(self, state, u, Fd=np.array([[0.0, 0.0]]), dt=None):
        if dt is None:
            dt = self.dt

        k1 = self.dynamics(0, state, u, Fd)
        k2 = self.dynamics(0, state + 0.5 * dt * k1, u, Fd)
        k3 = self.dynamics(0, state + 0.5 * dt * k2, u, Fd)
        k4 = self.dynamics(0, state + dt * k3, u, Fd)

        return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


    def simulate(self, state, controller, steps, target_pos=0.0):
        trajectory = np.zeros((steps, len(state)))
        trajectory[0] = state

        for t in range(1, steps):

            u = controller.get_action(state, target_pos)

            Fd = np.array([[0.0, 0.0]])

            if self.mode == "1":

                # Random event every 500 steps after t=2000
                if t > 2000 and t % 500 == 0:
                    self.apply_disturbance = (
                        np.random.rand() < 0.75 * self.disturbance_level / 5.0
                    )

                # If disturbance chosen, apply one impulse
                if self.apply_disturbance:
                    Fd = np.random.uniform(-3500, 3500, size=(1,2))
                    self.apply_disturbance = False
                    print(f"Disturbance applied at step {t}: Fd_x={Fd[0,0]:.2f}, Fd_theta={Fd[0,1]:.2f}")

            state = self.step(state, u, Fd)

            trajectory[t] = state
            if t == 1:
                disturbance_log = []

            disturbance_log.append(Fd)


        return trajectory, disturbance_log

    
    
    def disturbance(self, state, magnitude=0.1):
        disturbance = np.zeros(state.shape)

        choice = np.random.choice([1, 3])  # 0 for x, 2 for theta

        disturbance[choice] = np.random.uniform(-magnitude, magnitude)
        return state + disturbance

    def get_noisy_observation(self, state, std_dev=None):
        """Return the state with sensor noise added.
        Used by controller to simulate imperfect sensor readings.
        Actual dynamics remain clean.
        """
        if std_dev is None:
            std_dev = self.noise_std_dev
        noise = np.random.normal(0, std_dev, size=state.shape)
        return state + noise
        
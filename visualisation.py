# import matplotlib.pyplot as plt
# import numpy as np


# class Visualisation:

#     def __init__(self):
#         pass

#     def setup_figure(self):
#         pass

#     def plot_data(self):
#         pass

#     def animate(self):
#         pass

#     def kill_josh(self):
#         pass


import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
import numpy as np
from pendulum import Pendulum
from controllers import PIDController, LQRController


def visualize_trajectory(pendulum, trajectory, save_gif=False):
    """
    Visualize the pendulum trajectory with animation and plots
    
    Args:
        pendulum: Pendulum object with parameters
        trajectory: numpy array of shape (steps, 4) with [x, x_dot, theta, theta_dot]
        save_gif: whether to save animation as gif
    """
    
    # Extract data
    trajectory, disturbance_log = trajectory
    x = trajectory[:, 0]
    x_dot = trajectory[:, 1]
    theta = trajectory[:, 2]
    theta_dot = trajectory[:, 3]
    time = np.arange(len(trajectory)) * pendulum.dt
    
    # Create figure with subplots
    fig = plt.figure(figsize=(15, 10))
    
    # Animation subplot
    ax_anim = plt.subplot(2, 2, 1)
    ax_anim.set_xlim(-3, 3)
    ax_anim.set_ylim(-1, 4)
    ax_anim.set_aspect('equal')
    ax_anim.grid(True, alpha=0.3)
    ax_anim.set_title('Pendulum Animation')
    ax_anim.set_xlabel('x (m)')
    ax_anim.set_ylabel('y (m)')
    
    # Cart and pendulum elements
    cart_width = 0.3
    cart_height = 0.2
    cart = Rectangle((0, 0), cart_width, cart_height, fill=True, color='blue', alpha=0.7)
    ax_anim.add_patch(cart)
    
    pendulum_line, = ax_anim.plot([], [], 'o-', lw=3, color='red', markersize=10)
    trace, = ax_anim.plot([], [], 'g-', alpha=0.3, lw=1)
    time_text = ax_anim.text(0.02, 0.95, '', transform=ax_anim.transAxes)
    

    # Disturbance arrow (as a simple line for easy updating)
    disturbance_line, = ax_anim.plot([], [], color='purple', lw=4, alpha=0.0)

    # Position plot
    ax_pos = plt.subplot(2, 2, 2)
    ax_pos.plot(time, x, 'b-', label='Cart Position')
    ax_pos.set_xlabel('Time (s)')
    ax_pos.set_ylabel('Position (m)')
    ax_pos.set_title('Cart Position vs Time')
    ax_pos.grid(True, alpha=0.3)
    ax_pos.legend()
    
    # Angle plot
    ax_angle = plt.subplot(2, 2, 3)
    ax_angle.plot(time, np.degrees(theta), 'r-', label='Angle')
    ax_angle.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax_angle.set_xlabel('Time (s)')
    ax_angle.set_ylabel('Angle (degrees)')
    ax_angle.set_title('Pendulum Angle vs Time')
    ax_angle.grid(True, alpha=0.3)
    ax_angle.legend()
    
    # Velocities plot
    ax_vel = plt.subplot(2, 2, 4)
    ax_vel.plot(time, x_dot, 'b-', label='Cart Velocity', alpha=0.7)
    ax_vel.plot(time, theta_dot, 'r-', label='Angular Velocity', alpha=0.7)
    ax_vel.set_xlabel('Time (s)')
    ax_vel.set_ylabel('Velocity')
    ax_vel.set_title('Velocities vs Time')
    ax_vel.grid(True, alpha=0.3)
    ax_vel.legend()
    
    plt.tight_layout()
    
    # Animation update function
    trace_x, trace_y = [], []

    # Keep last N disturbance values for fade-out behavior
    ARROW_DECAY_FRAMES = 5000   # stays visible for ~50 frames (~0.05 sec at dt=0.001)
    disturbance_memory = np.zeros(ARROW_DECAY_FRAMES)

    
    def init():
        cart.set_xy((x[0] - cart_width/2, -cart_height/2))
        pendulum_line.set_data([], [])
        trace.set_data([], [])
        time_text.set_text('')
        disturbance_line.set_data([], [])
        disturbance_line.set_alpha(0.0)
        return cart, pendulum_line, trace, time_text, disturbance_line

    def animate(i):
    # Update cart position
        cart.set_xy((x[i] - cart_width/2, -cart_height/2))

        # Update pendulum
        pend_x = x[i] + pendulum.l * np.sin(theta[i])
        pend_y = pendulum.l * np.cos(theta[i])
        pendulum_line.set_data([x[i], pend_x], [0, pend_y])

        # Update trace
        trace_x.append(pend_x)
        trace_y.append(pend_y)
        trace.set_data(trace_x, trace_y)

        # Update time
        time_text.set_text(f'Time = {time[i]:.2f}s')

        # Shift disturbance memory
        disturbance_memory[:-1] = disturbance_memory[1:]
        disturbance_memory[-1] = disturbance_log[i]   # new disturbance enters buffer

        # Compute “effective disturbance” as the maximum absolute recent force
        Fd_eff = disturbance_memory.max() if abs(disturbance_memory).max() > 1e-6 else 0.0
        sign_eff = np.sign(disturbance_memory[np.argmax(np.abs(disturbance_memory))]) if Fd_eff != 0 else 0
        Fd_eff *= sign_eff  # restore sign


        if abs(Fd_eff) > 1e-6:
            arrow_length = 0.5 * np.tanh(abs(Fd_eff) / 30)
            direction = np.sign(Fd_eff)

            x0 = x[i]
            y0 = 0.4
            x1 = x0 + direction * arrow_length
            y1 = y0

            disturbance_line.set_data([x0, x1], [y0, y1])

            # Fade out based on how old the max disturbance is
            age = ARROW_DECAY_FRAMES - np.argmax(disturbance_memory[::-1])
            alpha = age / ARROW_DECAY_FRAMES
            disturbance_line.set_alpha(alpha)
        else:
            disturbance_line.set_alpha(0.0)

            disturbance_line.set_alpha(0.0)

        disturbance_line.set_data([], [])
        disturbance_line.set_alpha(0.0)
        return cart, pendulum_line, trace, time_text, disturbance_line

    
    # Create animation
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=len(trajectory), interval=pendulum.dt*1000,
                                   blit=True, repeat=True)
    
    if save_gif:
        anim.save('pendulum_simulation.gif', writer='pillow', fps=int(1/pendulum.dt))
    
    plt.show()
    
    return fig, anim


# Simple static plot (no animation)
def plot_results(pendulum, trajectory):
    """Simple static plots of the simulation results"""
    time = np.arange(len(trajectory)) * pendulum.dt
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    # Cart position
    axes[0, 0].plot(time, trajectory[:, 0], 'b-')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Cart Position (m)')
    axes[0, 0].set_title('Cart Position')
    axes[0, 0].grid(True, alpha=0.3)
    
    # Cart velocity
    axes[0, 1].plot(time, trajectory[:, 1], 'g-')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Cart Velocity (m/s)')
    axes[0, 1].set_title('Cart Velocity')
    axes[0, 1].grid(True, alpha=0.3)
    
    # Pendulum angle
    axes[1, 0].plot(time, np.degrees(trajectory[:, 2]), 'r-')
    axes[1, 0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Angle (degrees)')
    axes[1, 0].set_title('Pendulum Angle')
    axes[1, 0].grid(True, alpha=0.3)
    
    # Angular velocity
    axes[1, 1].plot(time, trajectory[:, 3], 'm-')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Angular Velocity (rad/s)')
    axes[1, 1].set_title('Angular Velocity')
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    return fig


# Example usage:
if __name__ == "__main__":
    # Create pendulum
    pend = Pendulum(M=0.5, m=0.2, l=0.8, b=0.1, dt=0.001, mode="1", disturbance_level=2)
    
    # Create a simple controller (example: constant force or proportional control)
    class SimpleController:
        def get_action(self, state, target_pos):
            # Simple proportional control
            Kp = 10
            return 0
    
    controller = PIDController(kp_theta=75.0, kd_theta=2, ki_theta=0,kp_x=0, kd_x=0, ki_x=0)
    
    #controller = controller = LQRController(M=0.5, m=0.2, l=0.8, b=0.1, Q=np.diag([10.0, 1.0, 100.0, 1.0]), R=0.1)
    Q = np.diag([10.0, 1.0, 100.0, 1.0])
    R = np.array([[0.01]])
    controller = LQRController(M=0.5, m=0.2, l=0.8, b=0.1, Q=Q, R=R)
    # controller = SimpleController()
    
    # Initial state: [x, x_dot, theta, theta_dot]
    initial_state = np.array([0.0, 0.0, 0.2, 0.0])  # Start with small angle
    
    # Simulate
    trajectory = pend.simulate(initial_state, controller, steps=10000, target_pos=[0.0, 0.0])
    
    # Visualize
    # Option 1: Animated
    visualize_trajectory(pend, trajectory, save_gif=False)
    
    # Option 2: Static plots only
    # plot_results(pend, trajectory)
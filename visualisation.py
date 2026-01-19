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
from controllers import PIDController


def visualize_trajectory(pendulum, trajectory, save_gif=False):
    """
    Visualize the pendulum trajectory with animation and plots
    
    Args:
        pendulum: Pendulum object with parameters
        trajectory: numpy array of shape (steps, 4) with [x, x_dot, theta, theta_dot]
        save_gif: whether to save animation as gif
    """
    
    # Extract data
    x = trajectory[:, 0]
    x_dot = trajectory[:, 1]
    theta = trajectory[:, 2]
    theta_dot = trajectory[:, 3]
    time = np.arange(len(trajectory)) * pendulum.dt
    
    # Create figure with subplots
    fig = plt.figure(figsize=(15, 10))
    
    # Animation subplot
    ax_anim = plt.subplot(2, 2, 1)
    ax_anim.set_xlim(-2, 2)
    ax_anim.set_ylim(-1.5, 1.5)
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
    
    def init():
        cart.set_xy((x[0] - cart_width/2, -cart_height/2))
        pendulum_line.set_data([], [])
        trace.set_data([], [])
        time_text.set_text('')
        return cart, pendulum_line, trace, time_text
    
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
        
        return cart, pendulum_line, trace, time_text
    
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
    pend = Pendulum(M=0.5, m=0.2, l=0.8, b=0.1, dt=0.001)
    
    # Create a simple controller (example: constant force or proportional control)
    class SimpleController:
        def get_action(self, state, target_pos):
            # Simple proportional control
            Kp = 10
            return 0
    
    controller = PIDController(kp_theta=50.0, kd_theta=1, ki_theta=0,
                               kp_x=0, kd_x=0, ki_x=0)
    # controller = SimpleController()
    
    # Initial state: [x, x_dot, theta, theta_dot]
    initial_state = np.array([0.0, 0.0, 0.2, 0.0])  # Start with small angle
    
    # Simulate
    trajectory = pend.simulate(initial_state, controller, steps=50000, target_pos=0.0)
    
    # Visualize
    # Option 1: Animated
    visualize_trajectory(pend, trajectory, save_gif=False)
    
    # Option 2: Static plots only
    # plot_results(pend, trajectory)
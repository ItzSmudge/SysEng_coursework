import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from matplotlib.widgets import Slider, Button
import numpy as np
from pendulum import Pendulum
from controllers import PIDController, LQRController
import threading
import queue


def visualize_trajectory_interactive(pendulum, initial_state, controller, steps=10000, target_pos=[0.0, 0.0], save_gif=False):
    """
    Interactive visualization with parameter sliders based on controller type
    """
    
    controller_type = type(controller).__name__
    
    # Shared data structures
    sim_data = {
        'trajectory': None,
        'disturbance_log': None,
        'lock': threading.Lock(),
        'update_ready': threading.Event(),
        'running': True
    }
    
    update_queue = queue.Queue()
    
    # Run initial simulation
    trajectory, disturbance_log = pendulum.simulate(initial_state.copy(), controller, steps, target_pos)
    with sim_data['lock']:
        sim_data['trajectory'] = trajectory
        sim_data['disturbance_log'] = disturbance_log
    
    # Create figure with proper spacing
    fig = plt.figure(figsize=(18, 9))
    fig.canvas.manager.set_window_title('Interactive Pendulum Simulator')
    
    # Main plot area (left 60% of window)
    # Animation (top left)
    ax_anim = plt.axes([0.05, 0.55, 0.25, 0.40])
    # Position (top middle)
    ax_pos = plt.axes([0.35, 0.55, 0.25, 0.40])
    # Angle (top right)  
    ax_angle = plt.axes([0.65, 0.55, 0.25, 0.40])
    # Velocities (bottom, spans width)
    ax_vel = plt.axes([0.05, 0.10, 0.85, 0.35])
    
    # Animation setup
    ax_anim.set_xlim(-3, 3)
    ax_anim.set_ylim(-1, 4)
    ax_anim.set_aspect('equal')
    ax_anim.grid(True, alpha=0.3)
    ax_anim.set_title(f'Animation ({controller_type})', fontsize=10, fontweight='bold')
    ax_anim.set_xlabel('Position (m)', fontsize=8)
    ax_anim.set_ylabel('Height (m)', fontsize=8)
    
    # Cart and pendulum
    cart_width = 0.3
    cart_height = 0.2
    cart = Rectangle((0, 0), cart_width, cart_height, fill=True, color='royalblue', alpha=0.8)
    ax_anim.add_patch(cart)
    
    pendulum_line, = ax_anim.plot([], [], 'o-', lw=3, color='crimson', markersize=10)
    trace, = ax_anim.plot([], [], '-', alpha=0.4, lw=1, color='green')
    time_text = ax_anim.text(0.02, 0.98, '', transform=ax_anim.transAxes, fontsize=9,
                             verticalalignment='top',
                             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    disturbance_line, = ax_anim.plot([], [], color='purple', lw=4, alpha=0.0)

    # Position plot - DYNAMIC
    line_pos, = ax_pos.plot([], [], 'b-', linewidth=1.5)
    ax_pos.set_xlabel('Time (s)', fontsize=8)
    ax_pos.set_ylabel('Position (m)', fontsize=8)
    ax_pos.set_title('Cart Position', fontsize=10, fontweight='bold')
    ax_pos.grid(True, alpha=0.3)
    ax_pos.set_xlim(0, 10)
    ax_pos.set_ylim(-2, 2)
    
    # Angle plot - DYNAMIC
    line_angle, = ax_angle.plot([], [], 'r-', linewidth=1.5)
    ax_angle.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax_angle.set_xlabel('Time (s)', fontsize=8)
    ax_angle.set_ylabel('Angle (°)', fontsize=8)
    ax_angle.set_title('Pendulum Angle', fontsize=10, fontweight='bold')
    ax_angle.grid(True, alpha=0.3)
    ax_angle.set_xlim(0, 10)
    ax_angle.set_ylim(-20, 20)
    
    # Velocities plot - DYNAMIC
    line_x_dot, = ax_vel.plot([], [], 'b-', label='Cart Velocity', linewidth=1.5)
    line_theta_dot, = ax_vel.plot([], [], 'r-', label='Angular Velocity', linewidth=1.5)
    ax_vel.set_xlabel('Time (s)', fontsize=8)
    ax_vel.set_ylabel('Velocity', fontsize=8)
    ax_vel.set_title('Velocities', fontsize=10, fontweight='bold')
    ax_vel.grid(True, alpha=0.3)
    ax_vel.legend(loc='upper right', fontsize=8)
    ax_vel.set_xlim(0, 10)
    ax_vel.set_ylim(-5, 5)
    
    # Slider area - RIGHT side, compact
    sliders = {}
    slider_left = 0.92
    slider_width = 0.06
    slider_height = 0.12
    slider_spacing = 0.14
    
    if controller_type == "PIDController":
        slider_start_y = 0.80
        
        # Vertical sliders for compactness
        configs = [
            ('kp_theta', 0, 200, controller.kp_theta, 'Kp θ'),
            ('kd_theta', 0, 50, controller.kd_theta, 'Kd θ'),
            ('ki_theta', 0, 10, controller.ki_theta, 'Ki θ'),
            ('kp_x', 0, 50, controller.kp_x, 'Kp x'),
            ('kd_x', 0, 50, controller.kd_x, 'Kd x'),
            ('ki_x', 0, 10, controller.ki_x, 'Ki x'),
        ]
        
        for i, (name, vmin, vmax, vinit, label) in enumerate(configs):
            row = i // 3
            col = i % 3
            ax_slider = plt.axes([slider_left - col*0.03, slider_start_y - row*0.35, 0.015, slider_height])
            slider = Slider(ax_slider, label, vmin, vmax, valinit=vinit, 
                          orientation='vertical', color='lightblue' if i < 3 else 'lightgreen')
            sliders[name] = slider
    
    elif controller_type == "LQRController":
        slider_start_y = 0.85
        
        configs = [
            ('q_x', 0.1, 500, controller.Q[0, 0], 'Q x'),
            ('q_x_dot', 0.1, 100, controller.Q[1, 1], 'Q v'),
            ('q_theta', 0.1, 1000, controller.Q[2, 2], 'Q θ'),
            ('q_theta_dot', 0.1, 100, controller.Q[3, 3], 'Q ω'),
            ('r', 0.001, 5, controller.R[0, 0], 'R'),
        ]
        
        for i, (name, vmin, vmax, vinit, label) in enumerate(configs):
            ax_slider = plt.axes([slider_left - (i % 3)*0.03, slider_start_y - (i // 3)*0.40, 0.015, slider_height])
            slider = Slider(ax_slider, label, vmin, vmax, valinit=vinit,
                          orientation='vertical', color='khaki' if i < 4 else 'salmon')
            sliders[name] = slider
    
    # Buttons - bottom right
    button_width = 0.06
    button_height = 0.04
    button_x_start = 0.915
    button_y = 0.02
    
    ax_play = plt.axes([button_x_start, button_y + 0.05, button_width, button_height])
    button_play = Button(ax_play, '▶', color='lightgreen', hovercolor='green')
    
    ax_pause = plt.axes([button_x_start, button_y, button_width, button_height])
    button_pause = Button(ax_pause, '⏸', color='yellow', hovercolor='gold')
    
    ax_reset = plt.axes([button_x_start - 0.065, button_y + 0.05, button_width, button_height])
    button_reset = Button(ax_reset, '↻', color='lightcoral', hovercolor='red')
    
    ax_apply = plt.axes([button_x_start - 0.065, button_y, button_width, button_height])
    button_apply = Button(ax_apply, '✓', color='lightskyblue', hovercolor='deepskyblue')
    
    # Animation state
    anim_state = {
        'paused': False,
        'frame': 0,
        'max_frame': len(trajectory)
    }
    
    # Plot data buffers
    plot_data = {
        'time': [],
        'x': [],
        'theta': [],
        'x_dot': [],
        'theta_dot': [],
        'max_time': 10.0
    }
    
    trace_x, trace_y = [], []
    ARROW_DECAY_FRAMES = 5000
    disturbance_memory = np.zeros(ARROW_DECAY_FRAMES)
    
    # Threading worker
    def simulation_worker():
        while sim_data['running']:
            try:
                params = update_queue.get(timeout=0.1)
                if params is None:
                    break
                
                print("⏳ Calculating new simulation...")
                
                if controller_type == "PIDController":
                    controller.set_kvalues(**params)
                elif controller_type == "LQRController":
                    Q = np.diag([params['q_x'], params['q_x_dot'], params['q_theta'], params['q_theta_dot']])
                    R = np.array([[params['r']]])
                    controller.Q = Q
                    controller.R = R
                    
                    M, m, l, b, I, g = controller.M, controller.m, controller.l, controller.b, controller.I, controller.g
                    denom = I * (M + m) + M * m * l**2
                    A = np.array([
                        [0, 1, 0, 0],
                        [0, -(I + m*l**2)*b / denom, (m**2 * l**2 * g) / denom, 0],
                        [0, 0, 0, 1],
                        [0, -(m*l*b) / denom, (m * l * g * (M + m)) / denom, 0]
                    ])
                    B = np.array([[0], [(I + m*l**2) / denom], [0], [(m*l) / denom]])
                    
                    import control as ct
                    controller.K, _, _ = ct.lqr(A, B, Q, R)
                    print(f"✓ K = {controller.K.flatten()}")
                
                controller.reset()
                new_trajectory, new_disturbance = pendulum.simulate(initial_state.copy(), controller, steps, target_pos)
                
                with sim_data['lock']:
                    sim_data['trajectory'] = new_trajectory
                    sim_data['disturbance_log'] = new_disturbance
                    sim_data['update_ready'].set()
                
                print("✓ Done! Press Play.")
                
            except queue.Empty:
                continue
    
    sim_thread = threading.Thread(target=simulation_worker, daemon=True)
    sim_thread.start()
    
    def play_clicked(event):
        anim_state['paused'] = False
    
    def pause_clicked(event):
        anim_state['paused'] = True
    
    def reset_clicked(event):
        anim_state['paused'] = True
        anim_state['frame'] = 0
        plot_data['time'].clear()
        plot_data['x'].clear()
        plot_data['theta'].clear()
        plot_data['x_dot'].clear()
        plot_data['theta_dot'].clear()
        trace_x.clear()
        trace_y.clear()
        disturbance_memory[:] = 0
        
        # Clear plot lines
        line_pos.set_data([], [])
        line_angle.set_data([], [])
        line_x_dot.set_data([], [])
        line_theta_dot.set_data([], [])
        
        fig.canvas.draw_idle()
        print("↻ Reset")
    
    def apply_changes(event):
        anim_state['paused'] = True
        anim_state['frame'] = 0
        plot_data['time'].clear()
        plot_data['x'].clear()
        plot_data['theta'].clear()
        plot_data['x_dot'].clear()
        plot_data['theta_dot'].clear()
        
        params = {name: slider.val for name, slider in sliders.items()}
        update_queue.put(params)
        print("✓ Applying...")
    
    button_play.on_clicked(play_clicked)
    button_pause.on_clicked(pause_clicked)
    button_reset.on_clicked(reset_clicked)
    button_apply.on_clicked(apply_changes)
    
    def init():
        cart.set_xy((0 - cart_width/2, -cart_height/2))
        pendulum_line.set_data([], [])
        trace.set_data([], [])
        time_text.set_text('')
        disturbance_line.set_data([], [])
        line_pos.set_data([], [])
        line_angle.set_data([], [])
        line_x_dot.set_data([], [])
        line_theta_dot.set_data([], [])
        return (cart, pendulum_line, trace, time_text, disturbance_line,
                line_pos, line_angle, line_x_dot, line_theta_dot)

    def animate(i):
        # Check for updates
        if sim_data['update_ready'].is_set():
            with sim_data['lock']:
                trajectory = sim_data['trajectory'].copy()
                disturbance_log = sim_data['disturbance_log'].copy()
                sim_data['update_ready'].clear()
            
            anim_state['max_frame'] = len(trajectory)
            anim_state['frame'] = 0
            anim_state['paused'] = True
            
            plot_data['time'].clear()
            plot_data['x'].clear()
            plot_data['theta'].clear()
            plot_data['x_dot'].clear()
            plot_data['theta_dot'].clear()
            trace_x.clear()
            trace_y.clear()
            
            # Store new trajectory
            with sim_data['lock']:
                sim_data['trajectory'] = trajectory
                sim_data['disturbance_log'] = disturbance_log
            
            print("✓ Loaded. Press Play.")
        
        if anim_state['paused']:
            return (cart, pendulum_line, trace, time_text, disturbance_line,
                    line_pos, line_angle, line_x_dot, line_theta_dot)
        
        # Get current trajectory
        with sim_data['lock']:
            trajectory = sim_data['trajectory']
            disturbance_log = sim_data['disturbance_log']
        
        idx = anim_state['frame'] % anim_state['max_frame']
        anim_state['frame'] += 1
        
        x_val = trajectory[idx, 0]
        x_dot_val = trajectory[idx, 1]
        theta_val = trajectory[idx, 2]
        theta_dot_val = trajectory[idx, 3]
        time_val = idx * pendulum.dt
        
        # Update cart
        cart.set_xy((x_val - cart_width/2, -cart_height/2))
        
        # Update pendulum
        pend_x = x_val + pendulum.l * np.sin(theta_val)
        pend_y = pendulum.l * np.cos(theta_val)
        pendulum_line.set_data([x_val, pend_x], [0, pend_y])
        
        # Update trace
        trace_x.append(pend_x)
        trace_y.append(pend_y)
        if len(trace_x) > 1000:
            trace_x.pop(0)
            trace_y.pop(0)
        trace.set_data(trace_x, trace_y)
        
        time_text.set_text(f't={time_val:.2f}s')
        
        # Update DYNAMIC plots
        plot_data['time'].append(time_val)
        plot_data['x'].append(x_val)
        plot_data['theta'].append(np.degrees(theta_val))
        plot_data['x_dot'].append(x_dot_val)
        plot_data['theta_dot'].append(theta_dot_val)
        
        # Keep only visible window
        if time_val > plot_data['max_time']:
            plot_data['max_time'] = time_val + 5
            ax_pos.set_xlim(time_val - 10, time_val + 2)
            ax_angle.set_xlim(time_val - 10, time_val + 2)
            ax_vel.set_xlim(time_val - 10, time_val + 2)
        
        line_pos.set_data(plot_data['time'], plot_data['x'])
        line_angle.set_data(plot_data['time'], plot_data['theta'])
        line_x_dot.set_data(plot_data['time'], plot_data['x_dot'])
        line_theta_dot.set_data(plot_data['time'], plot_data['theta_dot'])
        
        # Auto-scale y axes
        if len(plot_data['x']) > 10:
            ax_pos.set_ylim(min(plot_data['x'][-100:]) - 0.5, max(plot_data['x'][-100:]) + 0.5)
            ax_angle.set_ylim(min(plot_data['theta'][-100:]) - 5, max(plot_data['theta'][-100:]) + 5)
            y_min = min(min(plot_data['x_dot'][-100:]), min(plot_data['theta_dot'][-100:])) - 1
            y_max = max(max(plot_data['x_dot'][-100:]), max(plot_data['theta_dot'][-100:])) + 1
            ax_vel.set_ylim(y_min, y_max)
        
        # Disturbance
        disturbance_memory[:-1] = disturbance_memory[1:]
        disturbance_memory[-1] = disturbance_log[idx]
        
        Fd_eff = disturbance_memory.max() if abs(disturbance_memory).max() > 1e-6 else 0.0
        if Fd_eff != 0:
            sign_eff = np.sign(disturbance_memory[np.argmax(np.abs(disturbance_memory))])
            Fd_eff *= sign_eff
        
        if abs(Fd_eff) > 1e-6:
            arrow_len = 0.5 * np.tanh(abs(Fd_eff) / 30)
            direction = np.sign(Fd_eff)
            disturbance_line.set_data([x_val, x_val + direction * arrow_len], [0.4, 0.4])
            age = ARROW_DECAY_FRAMES - np.argmax(disturbance_memory[::-1])
            disturbance_line.set_alpha(min(1.0, age / ARROW_DECAY_FRAMES))
        else:
            disturbance_line.set_alpha(0.0)
        
        return (cart, pendulum_line, trace, time_text, disturbance_line,
                line_pos, line_angle, line_x_dot, line_theta_dot)
    
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=None, interval=pendulum.dt*1000,
                                   blit=True, repeat=True, cache_frame_data=False)
    
    def on_close(event):
        sim_data['running'] = False
        update_queue.put(None)
        sim_thread.join(timeout=1)
    
    fig.canvas.mpl_connect('close_event', on_close)
    
    if save_gif:
        anim.save('pendulum_simulation.gif', writer='pillow', fps=int(1/pendulum.dt))
    
    plt.show()
    
    return fig, anim


if __name__ == "__main__":
    pend = Pendulum(M=0.5, m=0.2, l=0.8, b=0.1, dt=0.001, mode="1", disturbance_level=2)
    
    controller = PIDController(kp_theta=75.0, kd_theta=2, ki_theta=0, kp_x=0, kd_x=0, ki_x=0)
    
    # Q = np.diag([10.0, 1.0, 100.0, 1.0])
    # R = np.array([[0.01]])
    # controller = LQRController(M=0.5, m=0.2, l=0.8, b=0.1, Q=Q, R=R)
    
    initial_state = np.array([0.0, 0.0, 0.2, 0.0])
    
    visualize_trajectory_interactive(pend, initial_state, controller, steps=10000, target_pos=[0.0, 0.0])
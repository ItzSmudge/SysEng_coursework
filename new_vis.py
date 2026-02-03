import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from matplotlib.widgets import Slider, Button
import numpy as np
from pendulum import Pendulum
from controllers import PIDController, LQRController, TrajectoryPIDController
import threading
import queue
import control as ct


def visualize_trajectory_interactive(pendulum, initial_state, controller, steps=10000, target_pos=[0.0, 0.0], save_gif=False):
    # Determine controller type
    controller_type = type(controller).__name__
    
    # Shared data structures
    sim_data = {
        'trajectory': None,
        'disturbance_log': None,
        'control_log': None,
        'lock': threading.Lock(),
        'update_ready': threading.Event(),
        'running': True
    }
    
    update_queue = queue.Queue()
    
    trajectory, disturbance_log = pendulum.simulate(initial_state.copy(), controller, steps, target_pos)

    control_log = []
    state = initial_state.copy()
    for t in range(steps):
        # Controller reads noisy observations
        noisy_state = pendulum.get_noisy_observation(state, std_dev=pendulum.noise_std_dev)
        u = controller.get_action(noisy_state, target_pos)
        control_log.append(u)
        Fd = np.array([[0.0, 0.0]])
        if t < len(disturbance_log):
            Fd = disturbance_log[t]
        state = pendulum.step(state, u, Fd)
    
    sim_data['trajectory'] = trajectory
    sim_data['disturbance_log'] = disturbance_log
    sim_data['control_log'] = np.array(control_log)
    
    # Extract initial data
    x = trajectory[:, 0].copy()
    x_dot = trajectory[:, 1].copy()
    theta = trajectory[:, 2].copy()
    theta_dot = trajectory[:, 3].copy()
    control_input = (sim_data['control_log'].copy() / 4)*0.8
    time = np.arange(len(trajectory)) * pendulum.dt
    
    print(control_input)

    # ============ WINDOW 1: Animation + Controls ============
    fig_anim = plt.figure(figsize=(16, 10))
    fig_anim.canvas.manager.set_window_title('Pendulum Animation & Controls')
    
    # Animation subplot - takes up most of the space
    ax_anim = fig_anim.add_axes([0.22, 0.35, 0.75, 0.60])
    ax_anim.set_xlim(-3, 3)
    ax_anim.set_ylim(-1, 4)
    ax_anim.set_aspect('equal')
    ax_anim.grid(True, alpha=0.3)
    ax_anim.set_title(f'Pendulum Animation\n({controller_type})', fontsize=12, fontweight='bold', pad=10)
    ax_anim.set_xlabel('Position (m)', fontsize=10)
    ax_anim.set_ylabel('Height (m)', fontsize=10)
    
    # Cart and pendulum elements
    cart_width = 0.3
    cart_height = 0.2
    cart = Rectangle((0, 0), cart_width, cart_height, fill=True, color='royalblue', alpha=0.8)
    ax_anim.add_patch(cart)
    
    pendulum_line, = ax_anim.plot([], [], 'o-', lw=3, color='crimson', markersize=12)
    trace, = ax_anim.plot([], [], '-', alpha=0.4, lw=1, color='green')
    time_text = ax_anim.text(0.02, 0.97, '', transform=ax_anim.transAxes, fontsize=11, 
                             verticalalignment='top',
                             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    disturbance_line, = ax_anim.plot([], [], color='purple', lw=5, alpha=0.0)
    
    # ============ LEFT PANEL: SIMULATION PARAMETERS ============
    # Simulation parameters on the left
    left_panel_x = 0.02
    sim_param_start_y = 0.9
    
    fig_anim.text(left_panel_x + 0.08, sim_param_start_y, 'Simulation Parameters', 
            fontsize=11, fontweight='bold', 
            bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8, pad=0.5))
    
    # Target Position slider (vertical, spaced below title)
    ax_target = fig_anim.add_axes([left_panel_x + 0.01, sim_param_start_y - 0.35, 0.04, 0.20])
    slider_target = Slider(ax_target, 'Target Pos', -3, 3, valinit=target_pos[0], color='orange', valstep=0.1, orientation='vertical')
    
    # Noise Level slider (vertical, next to Target Position)
    ax_noise = fig_anim.add_axes([left_panel_x + 0.07, sim_param_start_y - 0.35, 0.04, 0.20])
    slider_noise = Slider(ax_noise, 'Sensor Noise', 0.0, 1.0, valinit=pendulum.noise_std_dev, color='pink', valstep=0.01, orientation='vertical')
    
    # Disturbance Level slider (vertical, next to Noise Level)
    ax_disturbance = fig_anim.add_axes([left_panel_x + 0.13, sim_param_start_y - 0.35, 0.04, 0.20])
    slider_disturbance = Slider(ax_disturbance, 'Disturbance', 0, 5, valinit=pendulum.disturbance_level, color='plum', valstep=0.1, orientation='vertical')
    
    sliders = {}
    sliders['target_pos'] = slider_target
    sliders['noise_std_dev'] = slider_noise
    sliders['disturbance_level'] = slider_disturbance
    
    # Sliders area for controller parameters
    slider_left = 0.2
    slider_width = 0.75
    slider_height = 0.015
    slider_spacing = 0.025
    
    if controller_type == "PIDController" or controller_type == "TrajectoryPIDController":
        slider_start_y = 0.2
        
        fig_anim.text(0.5, slider_start_y + 0.03, 'Angle Control', 
                ha='center', fontsize=10, fontweight='bold', 
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7, pad=0.5))
        
        slider_configs = [
            ('kp_theta', 0, 200, controller.kp_theta, 'Kp θ', 'lightblue'),
            ('kd_theta', 0, 50, controller.kd_theta, 'Kd θ', 'lightblue'),
            ('ki_theta', 0, 10, controller.ki_theta, 'Ki θ', 'lightblue'),
        ]
        
        for i, (name, vmin, vmax, vinit, label, color) in enumerate(slider_configs):
            ax_slider = plt.axes([slider_left, slider_start_y - (i+1) * slider_spacing, slider_width, slider_height])
            slider = Slider(ax_slider, label, vmin, vmax, valinit=vinit, color=color, valstep=(vmax-vmin)/1000)
            sliders[name] = slider
        
        position_start_y = slider_start_y - 4.5 * slider_spacing
        fig_anim.text(0.5, position_start_y + 0.03, 'Position Control', 
                ha='center', fontsize=10, fontweight='bold', 
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7, pad=0.5))
        
        slider_configs_x = [
            ('kp_x', 0, 50, controller.kp_x, 'Kp x', 'lightgreen'),
            ('kd_x', 0, 50, controller.kd_x, 'Kd x', 'lightgreen'),
            ('ki_x', 0, 10, controller.ki_x, 'Ki x', 'lightgreen'),
        ]
        
        for i, (name, vmin, vmax, vinit, label, color) in enumerate(slider_configs_x):
            ax_slider = plt.axes([slider_left, position_start_y - (i+1) * slider_spacing, slider_width, slider_height])
            slider = Slider(ax_slider, label, vmin, vmax, valinit=vinit, color=color, valstep=(vmax-vmin)/1000)
            sliders[name] = slider
    
    elif controller_type == "LQRController":
        slider_start_y = 0.2
        
        fig_anim.text(0.5, slider_start_y + 0.03, 'State Cost Matrix Q', 
                ha='center', fontsize=10, fontweight='bold', 
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.7, pad=0.5))
        
        slider_configs = [
            ('q_x', 0.1, 500, controller.Q[0, 0], 'Q[0,0] Position', 'khaki'),
            ('q_x_dot', 0.1, 100, controller.Q[1, 1], 'Q[1,1] Velocity', 'khaki'),
            ('q_theta', 0.1, 1000, controller.Q[2, 2], 'Q[2,2] Angle', 'khaki'),
            ('q_theta_dot', 0.1, 100, controller.Q[3, 3], 'Q[3,3] Ang Vel', 'khaki'),
        ]
        
        for i, (name, vmin, vmax, vinit, label, color) in enumerate(slider_configs):
            ax_slider = plt.axes([slider_left, slider_start_y - (i+1) * slider_spacing, slider_width, slider_height])
            slider = Slider(ax_slider, label, vmin, vmax, valinit=vinit, color=color)
            sliders[name] = slider
        
        r_start_y = slider_start_y - 5.5 * slider_spacing
        fig_anim.text(0.5, r_start_y + 0.02, 'Control Cost R', 
                ha='center', fontsize=10, fontweight='bold', 
                bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.7, pad=0.5))
        
        ax_slider = plt.axes([slider_left, r_start_y - slider_spacing, slider_width, slider_height])
        slider = Slider(ax_slider, 'R Control Effort', 0.001, 5, valinit=controller.R[0, 0], color='salmon')
        sliders['r'] = slider
    
    # Control buttons
    button_width = 0.10
    button_height = 0.03
    button_y = 0.02
    button_spacing = 0.12

    ax_play = plt.axes([0.35, button_y, button_width, button_height])
    button_play = Button(ax_play, '▶ Play', color='lightgreen', hovercolor='green')

    ax_pause = plt.axes([0.35 + button_spacing, button_y, button_width, button_height])
    button_pause = Button(ax_pause, '⏸ Pause', color='yellow', hovercolor='gold')

    ax_reset = plt.axes([0.35 + 2*button_spacing, button_y, button_width, button_height])
    button_reset = Button(ax_reset, '↻ Reset', color='lightcoral', hovercolor='red')

    ax_apply = plt.axes([0.35 + 3*button_spacing, button_y, button_width, button_height])
    button_apply = Button(ax_apply, '✓ Apply', color='lightskyblue', hovercolor='deepskyblue')
    
    # ============ WINDOW 2: Graphs ============
    fig_graphs = plt.figure(figsize=(14, 10))
    fig_graphs.canvas.manager.set_window_title('State & Control Graphs')
    
    # Create subplot grid: 2 rows x 2 columns
    ax_pos = plt.subplot(2, 2, 1)
    ax_angle = plt.subplot(2, 2, 2)
    ax_vel = plt.subplot(2, 2, 3)
    ax_control = plt.subplot(2, 2, 4)
    
    # Position plot
    line_pos, = ax_pos.plot(time, x, 'b-', linewidth=1.5)
    current_pos_marker, = ax_pos.plot([], [], 'ro', markersize=8)
    ax_pos.set_xlabel('Time (s)', fontsize=9)
    ax_pos.set_ylabel('Position (m)', fontsize=9)
    ax_pos.set_title('Cart Position', fontsize=10, fontweight='bold')
    ax_pos.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    
    # Angle plot
    line_angle, = ax_angle.plot(time, np.degrees(theta), 'r-', linewidth=1.5)
    current_angle_marker, = ax_angle.plot([], [], 'ro', markersize=8)
    ax_angle.axhline(y=0, color='k', linestyle='--', alpha=0.5, linewidth=1)
    ax_angle.set_xlabel('Time (s)', fontsize=9)
    ax_angle.set_ylabel('Angle (°)', fontsize=9)
    ax_angle.set_title('Pendulum Angle', fontsize=10, fontweight='bold')
    ax_angle.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    
    # Velocities plot
    line_x_dot, = ax_vel.plot(time, x_dot, 'b-', label='Cart Velocity', alpha=0.8, linewidth=1.5)
    line_theta_dot, = ax_vel.plot(time, theta_dot, 'r-', label='Angular Velocity', alpha=0.8, linewidth=1.5)
    current_vel_marker, = ax_vel.plot([], [], 'bo', markersize=8)
    current_ang_vel_marker, = ax_vel.plot([], [], 'ro', markersize=8)
    ax_vel.set_xlabel('Time (s)', fontsize=9)
    ax_vel.set_ylabel('Velocity', fontsize=9)
    ax_vel.set_title('Velocities', fontsize=10, fontweight='bold')
    ax_vel.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    ax_vel.legend(loc='upper right', fontsize=8)
    
    # Control input plot
    line_control, = ax_control.plot(time, control_input, 'g-', linewidth=1.5)
    current_control_marker, = ax_control.plot([], [], 'ro', markersize=8)
    ax_control.axhline(y=0, color='k', linestyle='--', alpha=0.5, linewidth=1)
    ax_control.set_xlabel('Time (s)', fontsize=9)
    ax_control.set_ylabel('Voltage (V)', fontsize=9)
    ax_control.set_title('Control Input', fontsize=10, fontweight='bold')
    ax_control.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    
    plt.tight_layout()
    
    # Animation state
    anim_state = {
        'paused': False,
        'frame': 0
    }
    
    trace_x, trace_y = [], []
    ARROW_DECAY_FRAMES = 5000
    disturbance_memory = np.zeros(ARROW_DECAY_FRAMES)
    
    def update_plot_display(idx):
        """Update all plots to show state at given frame"""
        # Update position marker
        current_pos_marker.set_data([time[idx]], [x[idx]])
        
        # Update angle marker
        current_angle_marker.set_data([time[idx]], [np.degrees(theta[idx])])
        
        # Update velocity markers
        current_vel_marker.set_data([time[idx]], [x_dot[idx]])
        current_ang_vel_marker.set_data([time[idx]], [theta_dot[idx]])
        
        # Update control marker
        current_control_marker.set_data([time[idx]], [control_input[idx]])
    
    # Threading worker for simulation
    def simulation_worker():
        while sim_data['running']:
            try:
                params = update_queue.get(timeout=0.1)
                if params is None:
                    break
                
                print("Running simulation with new parameters...")
                
                # Extract simulation parameters
                current_target_pos = params.get('target_pos', target_pos[0])
                current_noise_std = params.get('noise_std_dev', pendulum.noise_std_dev)
                current_disturbance = params.get('disturbance_level', pendulum.disturbance_level)
                
                # Update pendulum simulation parameters
                pendulum.noise_std_dev = current_noise_std
                pendulum.disturbance_level = current_disturbance
                
                # Update controller with new parameters (excluding simulation params)
                controller_params = {k: v for k, v in params.items() 
                                    if k not in ['target_pos', 'noise_std_dev', 'disturbance_level']}
                
                if controller_type == "PIDController" or controller_type == "TrajectoryPIDController":
                    controller.set_kvalues(**controller_params)
                elif controller_type == "LQRController":
                    Q = np.diag([params['q_x'], params['q_x_dot'], params['q_theta'], params['q_theta_dot']])
                    R = np.array([[params['r']]])
                    controller.Q = Q
                    controller.R = R
                    
                    # Recompute LQR gains
                    M, m, l, b, I, g = controller.M, controller.m, controller.l, controller.b, controller.I, controller.g
                    denom = I * (M + m) + M * m * l**2
                    A = np.array([
                        [0, 1, 0, 0],
                        [0, -(I + m*l**2)*b / denom, (m**2 * l**2 * g) / denom, 0],
                        [0, 0, 0, 1],
                        [0, -(m*l*b) / denom, (m * l * g * (M + m)) / denom, 0]
                    ])
                    B = np.array([[0], [(I + m*l**2) / denom], [0], [(m*l) / denom]])
                    
                    controller.K, _, _ = ct.lqr(A, B, Q, R)
                    print(f"✓ LQR gains: K = {controller.K.flatten()}")
                
                # Reset and re-run simulation with new target position
                controller.reset()
                new_target = [current_target_pos, 0.0]
                new_trajectory, new_disturbance_log = pendulum.simulate(initial_state.copy(), controller, steps, new_target)
                
                # Re-compute control inputs
                new_control_log = []
                state = initial_state.copy()
                for t in range(steps):
                    # Controller reads noisy observations
                    noisy_state = pendulum.get_noisy_observation(state, std_dev=current_noise_std)
                    u = controller.get_action(noisy_state, new_target)
                    new_control_log.append(u)
                    Fd = np.array([[0.0, 0.0]])
                    if t < len(new_disturbance_log):
                        Fd = new_disturbance_log[t]
                    state = pendulum.step(state, u, Fd)
                
                # Update shared data
                with sim_data['lock']:
                    sim_data['trajectory'] = new_trajectory
                    sim_data['disturbance_log'] = new_disturbance_log
                    sim_data['control_log'] = np.array(new_control_log)
                    sim_data['update_ready'].set()
                
                print("✓ Simulation complete! Press Play to start.")
                
            except queue.Empty:
                continue
    
    # Start simulation worker thread
    sim_thread = threading.Thread(target=simulation_worker, daemon=True)
    sim_thread.start()
    
    def play_clicked(event):
        anim_state['paused'] = False
        print("▶ Playing")
    
    def pause_clicked(event):
        anim_state['paused'] = True
        print("⏸ Paused at t={:.2f}s".format(time[anim_state['frame']]))
   
    def reset_clicked(event):
        """Reset simulation with current slider values, reset to t=0, and pause"""
        anim_state['paused'] = True
        
        # Get current slider values and apply them
        params = {name: slider.val for name, slider in sliders.items()}
        
        print("↻ Resetting with current slider values...")
        
        # Extract simulation parameters
        current_target_pos = params.get('target_pos', target_pos[0])
        current_noise_std = params.get('noise_std_dev', pendulum.noise_std_dev)
        current_disturbance = params.get('disturbance_level', pendulum.disturbance_level)
        
        # Update pendulum simulation parameters
        pendulum.noise_std_dev = current_noise_std
        pendulum.disturbance_level = current_disturbance
        
        # Update controller with new parameters (excluding simulation params)
        controller_params = {k: v for k, v in params.items() 
                            if k not in ['target_pos', 'noise_std_dev', 'disturbance_level']}
        
        # Update controller with current slider parameters
        if controller_type == "PIDController" or controller_type == "TrajectoryPIDController":
            controller.set_kvalues(**controller_params)
        elif controller_type == "LQRController":
            Q = np.diag([params['q_x'], params['q_x_dot'], params['q_theta'], params['q_theta_dot']])
            R = np.array([[params['r']]])
            controller.Q = Q
            controller.R = R
            
            # Recompute LQR gains
            M, m, l, b, I, g = controller.M, controller.m, controller.l, controller.b, controller.I, controller.g
            denom = I * (M + m) + M * m * l**2
            A = np.array([
                [0, 1, 0, 0],
                [0, -(I + m*l**2)*b / denom, (m**2 * l**2 * g) / denom, 0],
                [0, 0, 0, 1],
                [0, -(m*l*b) / denom, (m * l * g * (M + m)) / denom, 0]
            ])
            B = np.array([[0], [(I + m*l**2) / denom], [0], [(m*l) / denom]])
            
            controller.K, _, _ = ct.lqr(A, B, Q, R)
            print(f"LQR gains: K = {controller.K.flatten()}")
        
        # Reset and re-run simulation with new target position
        controller.reset()
        new_target = [current_target_pos, 0.0]
        new_trajectory, new_disturbance_log = pendulum.simulate(initial_state.copy(), controller, steps, new_target)
        
        # Re-compute control inputs
        new_control_log = []
        state = initial_state.copy()
        for t in range(steps):
            # Controller reads noisy observations
            noisy_state = pendulum.get_noisy_observation(state, std_dev=current_noise_std)
            u = controller.get_action(noisy_state, new_target)
            new_control_log.append(u)
            Fd = np.array([[0.0, 0.0]])
            if t < len(new_disturbance_log):
                Fd = new_disturbance_log[t]
            state = pendulum.step(state, u, Fd)
        
        # Update data
        nonlocal x, x_dot, theta, theta_dot, control_input
        x[:] = new_trajectory[:, 0]
        x_dot[:] = new_trajectory[:, 1]
        theta[:] = new_trajectory[:, 2]
        theta_dot[:] = new_trajectory[:, 3]
        control_input[:] = new_control_log
        
        # Update disturbance log
        with sim_data['lock']:
            sim_data['trajectory'] = new_trajectory
            sim_data['disturbance_log'] = new_disturbance_log
            sim_data['control_log'] = np.array(new_control_log)
        
        # Clear markers first
        current_pos_marker.set_data([], [])
        current_angle_marker.set_data([], [])
        current_vel_marker.set_data([], [])
        current_ang_vel_marker.set_data([], [])
        current_control_marker.set_data([], [])
        
        # Update ALL plot lines (both x and y data)
        line_pos.set_data(time, x)
        line_angle.set_data(time, np.degrees(theta))
        line_x_dot.set_data(time, x_dot)
        line_theta_dot.set_data(time, theta_dot)
        line_control.set_data(time, control_input)
        
        # Rescale all axes
        ax_pos.relim()
        ax_pos.autoscale_view()
        ax_angle.relim()
        ax_angle.autoscale_view()
        ax_vel.relim()
        ax_vel.autoscale_view()
        ax_control.relim()
        ax_control.autoscale_view()
        
        # Reset animation state
        anim_state['frame'] = 0
        trace_x.clear()
        trace_y.clear()
        disturbance_memory[:] = 0
        
        # Update animation display
        cart.set_xy((x[0] - cart_width/2, -cart_height/2))
        pend_x = x[0] + pendulum.l * np.sin(-theta[0])
        pend_y = pendulum.l * np.cos(-theta[0])
        pendulum_line.set_data([x[0], pend_x], [0, pend_y])
        trace.set_data([], [])
        time_text.set_text(f'Time: {time[0]:.2f}s\nFrame: 0')
        disturbance_line.set_data([], [])
        disturbance_line.set_alpha(0.0)
        
        # Now update markers at t=0
        update_plot_display(0)
        
        # Force a complete redraw
        fig_anim.canvas.draw()
        fig_graphs.canvas.draw()
        print("✓ Reset complete with updated parameters. Press Play to start.")

    def apply_changes(event):
        anim_state['paused'] = True
        params = {name: slider.val for name, slider in sliders.items()}
        update_queue.put(params)
        print("Applying changes...")
    
    button_play.on_clicked(play_clicked)
    button_pause.on_clicked(pause_clicked)
    button_reset.on_clicked(reset_clicked)
    button_apply.on_clicked(apply_changes)
    
    def init():
        cart.set_xy((x[0] - cart_width/2, -cart_height/2))
        pendulum_line.set_data([], [])
        trace.set_data([], [])
        time_text.set_text('')
        disturbance_line.set_data([], [])
        disturbance_line.set_alpha(0.0)
        current_pos_marker.set_data([], [])
        current_angle_marker.set_data([], [])
        current_vel_marker.set_data([], [])
        current_ang_vel_marker.set_data([], [])
        current_control_marker.set_data([], [])
        return (cart, pendulum_line, trace, time_text, disturbance_line, 
                current_pos_marker, current_angle_marker, current_vel_marker, 
                current_ang_vel_marker, current_control_marker)

    def animate(i):
        nonlocal x, x_dot, theta, theta_dot, control_input
        
        # Check for simulation updates
        if sim_data['update_ready'].is_set():
            with sim_data['lock']:
                trajectory = sim_data['trajectory']
                disturbance_log = sim_data['disturbance_log']
                new_control_log = sim_data['control_log']
                sim_data['update_ready'].clear()
            
            # Update all data
            x[:] = trajectory[:, 0]
            x_dot[:] = trajectory[:, 1]
            theta[:] = trajectory[:, 2]
            theta_dot[:] = trajectory[:, 3]
            control_input[:] = new_control_log
            
            line_pos.set_ydata(x)
            line_angle.set_ydata(np.degrees(theta))
            line_x_dot.set_ydata(x_dot)
            line_theta_dot.set_ydata(theta_dot)
            line_control.set_ydata(control_input)
            
            ax_pos.relim()
            ax_pos.autoscale_view()
            ax_angle.relim()
            ax_angle.autoscale_view()
            ax_vel.relim()
            ax_vel.autoscale_view()
            ax_control.relim()
            ax_control.autoscale_view()
            
            # Reset to t=0 and pause
            anim_state['frame'] = 0
            anim_state['paused'] = True
            trace_x.clear()
            trace_y.clear()
            disturbance_memory[:] = 0
            
            # Update display to show t=0
            update_plot_display(0)
            
            print("✓ New simulation loaded at t=0. Press Play to start.")
        
        # Handle pause
        if anim_state['paused']:
            return (cart, pendulum_line, trace, time_text, disturbance_line, 
                    current_pos_marker, current_angle_marker, current_vel_marker, 
                    current_ang_vel_marker, current_control_marker)
        
        # Update frame
        idx = anim_state['frame'] % len(x)
        anim_state['frame'] += 1
        
        # Update cart
        cart.set_xy((x[idx] - cart_width/2, -cart_height/2))
        
        # Update pendulum
        pend_x = x[idx] + pendulum.l * np.sin(-theta[idx])
        pend_y = pendulum.l * np.cos(-theta[idx])
        pendulum_line.set_data([x[idx], pend_x], [0, pend_y])
        
        # Update trace
        trace_x.append(pend_x)
        trace_y.append(pend_y)
        if len(trace_x) > 2000:
            trace_x.pop(0)
            trace_y.pop(0)
        trace.set_data(trace_x, trace_y)
        
        # Update time
        time_text.set_text(f'Time: {time[idx]:.2f}s\nFrame: {idx}')
        
        # Update plot markers
        update_plot_display(idx)
        
        # Disturbance visualization
        disturbance_memory[:-1] = disturbance_memory[1:]
        if sim_data['disturbance_log'] is not None and idx < len(sim_data['disturbance_log']):
            current_Fd = sim_data['disturbance_log'][idx]
            if isinstance(current_Fd, np.ndarray) and current_Fd.size > 0:
                disturbance_memory[-1] = current_Fd.sum()
            else:
                disturbance_memory[-1] = 0
        else:
            disturbance_memory[-1] = 0

        Fd_eff = disturbance_memory.max() if abs(disturbance_memory).max() > 1e-6 else 0.0
        sign_eff = np.sign(disturbance_memory[np.argmax(np.abs(disturbance_memory))]) if Fd_eff != 0 else 0
        Fd_eff *= sign_eff

        if abs(Fd_eff) > 1e-6:
            arrow_length = 0.5 * np.tanh(abs(Fd_eff) / 3500)
            direction = np.sign(Fd_eff)
            x0, y0 = x[idx], 0.4
            x1, y1 = x0 + direction * arrow_length, y0
            disturbance_line.set_data([x0, x1], [y0, y1])
            age = ARROW_DECAY_FRAMES - np.argmax(disturbance_memory[::-1])
            alpha = min(1.0, age / ARROW_DECAY_FRAMES)
            disturbance_line.set_alpha(alpha)
        else:
            disturbance_line.set_alpha(0.0)
            disturbance_line.set_data([], [])
        
        return (cart, pendulum_line, trace, time_text, disturbance_line, 
                current_pos_marker, current_angle_marker, current_vel_marker, 
                current_ang_vel_marker, current_control_marker)
    
    # Create animation
    anim = animation.FuncAnimation(fig_anim, animate, init_func=init,
                                   frames=None, interval=5,
                                   blit=True, repeat=True, cache_frame_data=False)
    
    def on_close(event):
        sim_data['running'] = False
        update_queue.put(None)
        sim_thread.join(timeout=1)
    
    fig_anim.canvas.mpl_connect('close_event', on_close)
    fig_graphs.canvas.mpl_connect('close_event', on_close)
    
    if save_gif:
        anim.save('pendulum_simulation.gif', writer='pillow', fps=int(1/pendulum.dt))
    
    plt.show()
    
    return fig_anim, fig_graphs, anim


# Example usage:
if __name__ == "__main__":
    initial_state = np.array([0.0, 0.0, 0.1, 0.0])
    
    # LQR Controller
    Q = np.diag([10.0, 1.0, 100.0, 1.0])
    R = np.array([[0.01]])
    controller = LQRController(M=0.5, m=0.2, l=0.8, b=0.1, Q=Q, R=R, filter_enabled=True, window_size=50)
    # controller = PIDController(kp_x=0.0, kd_x=0.0, ki_x=0.0,
    #                            kp_theta=120.0, kd_theta=20.0, ki_theta=5.0)
    
    # Sprint
    pend = Pendulum(M=1.0, m=0.3, l=1.0, b=0.2, dt=0.001, mode="1", disturbance_level=0)

    visualize_trajectory_interactive(pend, initial_state, controller, steps=10000, target_pos=[0.0, 0.0])
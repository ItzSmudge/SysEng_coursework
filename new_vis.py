import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, FancyBboxPatch
from matplotlib.widgets import Slider, Button
import numpy as np

# --- UI Theme: Clean, modern palette ---
COLORS = {
    "bg": "#f8fafc",
    "panel": "#e2e8f0",
    "panel_dark": "#cbd5e1",
    "cart": "#3b82f6",
    "pendulum": "#dc2626",
    "trace": "#22c55e",
    "target": "#f59e0b",
    "disturbance": "#8b5cf6",
    "text": "#1e293b",
    "grid": "#94a3b8",
    "accent_green": "#10b981",
    "accent_blue": "#0ea5e9",
    "accent_amber": "#f59e0b",
}
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
        "trajectory": None,
        "disturbance_log": None,
        "control_log": None,
        "lock": threading.Lock(),
        "update_ready": threading.Event(),
        "running": True
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
    
    sim_data["trajectory"] = trajectory
    sim_data["disturbance_log"] = disturbance_log
    sim_data["control_log"] = np.array(control_log)
    
    # Extract initial data
    x = trajectory[:, 0].copy()
    x_dot = trajectory[:, 1].copy()
    theta = trajectory[:, 2].copy()
    theta_dot = trajectory[:, 3].copy()
    control_input = (sim_data["control_log"].copy() / 4)*0.8
    time = np.arange(len(trajectory)) * pendulum.dt

    # ============ WINDOW 1: Animation + Controls ============
    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except OSError:
        try:
            plt.style.use("seaborn-whitegrid")
        except OSError:
            pass
    fig_anim = plt.figure(figsize=(14, 8), facecolor=COLORS["bg"])
    fig_anim.canvas.manager.set_window_title("Pendulum Animation & Controls")

    # ============ LAYOUT: Left half = simulation, Right half = sliders ============
    # All positions in figure-normalized coords [left, bottom, width, height] — scales with window
    M = 0.025   # margin
    G = 0.012   # gap between halves
    LEFT_W = 0.5 - M - G / 2
    RIGHT_X = 0.5 + G / 2
    RIGHT_W = 0.5 - M - G / 2
    SLIDER_W = 0.28   # narrower sliders, centered in right panel

    # Solid box around simulation half
    box_pad = 0.006
    left_box = FancyBboxPatch((M - box_pad, M), LEFT_W + 2 * box_pad, 1 - 2 * M,
                              transform=fig_anim.transFigure, facecolor="white", edgecolor=COLORS["text"],
                              linewidth=2.5, boxstyle="round,pad=0.002", zorder=0)
    fig_anim.add_artist(left_box)

    # Left half: animation (inside box, spines reinforce border)
    ax_anim = fig_anim.add_axes([M, M, LEFT_W, 1 - 2 * M])
    ax_anim.set_facecolor("white")
    ax_anim.set_xlim(-3, 3)
    ax_anim.set_ylim(-1, 4)
    ax_anim.set_aspect("equal")
    ax_anim.grid(True, alpha=0.3, linestyle="-", linewidth=0.5, color=COLORS["grid"])
    ax_anim.set_title(f"Pendulum Animation — {controller_type}", fontsize=13, fontweight="bold", pad=12, color=COLORS["text"])
    ax_anim.set_xlabel("Position (m)", fontsize=10, color=COLORS["text"])
    ax_anim.set_ylabel("Height (m)", fontsize=10, color=COLORS["text"])
    for spine in ax_anim.spines.values():
        spine.set_color(COLORS["text"])
        spine.set_linewidth(2.5)

    # Target position indicator (dashed vertical line)
    target_line = ax_anim.axvline(x=target_pos[0], color=COLORS["target"], linestyle="--", alpha=0.8, linewidth=2, zorder=1)

    # Cart and pendulum elements
    cart_width = 0.3
    cart_height = 0.2
    cart = Rectangle((0, 0), cart_width, cart_height, fill=True, color=COLORS["cart"], alpha=0.9, ec=COLORS["panel_dark"], lw=1)
    ax_anim.add_patch(cart)

    pendulum_line, = ax_anim.plot([], [], "o-", lw=3.5, color=COLORS["pendulum"], markersize=11, markeredgecolor="white", markeredgewidth=1.5, zorder=5)
    trace, = ax_anim.plot([], [], "-", alpha=0.5, lw=1.5, color=COLORS["trace"], zorder=2)
    time_text = ax_anim.text(0.02, 0.97, "", transform=ax_anim.transAxes, fontsize=9,
                             verticalalignment="top", fontweight="500",
                             bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.85))
    disturbance_line, = ax_anim.plot([], [], color=COLORS["disturbance"], lw=5, alpha=0.0, zorder=4)

    # Solid box around right panel (sliders + buttons)
    right_box = FancyBboxPatch((RIGHT_X - box_pad, M), RIGHT_W + 2 * box_pad, 1 - 2 * M,
                               transform=fig_anim.transFigure, facecolor="white", edgecolor=COLORS["text"],
                               linewidth=2.5, boxstyle="round,pad=0.002", zorder=0)
    fig_anim.add_artist(right_box)

    # ============ RIGHT PANEL: Sliders + Buttons (compact, narrower sliders) ============
    SLIDER_LEFT = RIGHT_X + (RIGHT_W - SLIDER_W) / 2
    SH = 0.018
    SS = 0.022
    BW = 0.058
    BH = 0.035
    BS = 0.008

    def _style_slider(slider):
        """Make slider look clean and smooth: hide axis clutter, subtle track, rounded feel."""
        ax = slider.ax
        ax.set_facecolor("#f1f5f9")
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_color(COLORS["panel_dark"])
            spine.set_linewidth(0.8)
        # Style the filled polygon (track fill) - smoother look, no edge
        if hasattr(slider, "poly"):
            slider.poly.set_edgecolor("none")
            slider.poly.set_alpha(0.95)
        # Cleaner value display
        if hasattr(slider, "valtext"):
            slider.valtext.set_fontsize(8)
            slider.valtext.set_color(COLORS["text"])

    sliders = {}
    y = 0.96

    fig_anim.text(RIGHT_X + RIGHT_W / 2, y, "Simulation Parameters", ha="center", fontsize=9, fontweight="bold",
                  color=COLORS["text"], bbox=dict(boxstyle="round", facecolor=COLORS["panel"], alpha=0.9, pad=0.3))
    y -= 0.03
    ax_target = fig_anim.add_axes([SLIDER_LEFT, y - SH, SLIDER_W, SH])
    slider_target = Slider(ax_target, "Target Pos", -3, 3, valinit=target_pos[0], color=COLORS["target"], valstep=0.02)
    _style_slider(slider_target)
    sliders["target_pos"] = slider_target
    y -= SS
    ax_noise = fig_anim.add_axes([SLIDER_LEFT, y - SH, SLIDER_W, SH])
    slider_noise = Slider(ax_noise, "Sensor Noise", 0.0, 1.0, valinit=pendulum.noise_std_dev, color=COLORS["accent_blue"], valstep=0.005)
    _style_slider(slider_noise)
    sliders["noise_std_dev"] = slider_noise
    y -= SS
    ax_disturbance = fig_anim.add_axes([SLIDER_LEFT, y - SH, SLIDER_W, SH])
    slider_disturbance = Slider(ax_disturbance, "Disturbance", 0, 5, valinit=pendulum.disturbance_level, color=COLORS["disturbance"], valstep=0.05)
    _style_slider(slider_disturbance)
    y -= 0.04

    if controller_type == "PIDController" or controller_type == "TrajectoryPIDController":
        fig_anim.text(RIGHT_X + RIGHT_W / 2, y, "Angle Control", ha="center", fontsize=9, fontweight="bold",
                      color=COLORS["text"], bbox=dict(boxstyle="round", facecolor="#dbeafe", alpha=0.9, pad=0.3))
        y -= 0.025
        for name, vmin, vmax, vinit, label, color in [
            ("kp_theta", 0, 200, controller.kp_theta, "Kp θ", "#93c5fd"),
            ("kd_theta", 0, 50, controller.kd_theta, "Kd θ", "#93c5fd"),
            ("ki_theta", 0, 10, controller.ki_theta, "Ki θ", "#93c5fd"),
        ]:
            ax_s = fig_anim.add_axes([SLIDER_LEFT, y - SH, SLIDER_W, SH])
            s = Slider(ax_s, label, vmin, vmax, valinit=vinit, color=color, valstep=(vmax - vmin) / 2000)
            _style_slider(s)
            sliders[name] = s
            y -= SS
        y -= 0.02
        fig_anim.text(RIGHT_X + RIGHT_W / 2, y, "Position Control", ha="center", fontsize=9, fontweight="bold",
                      color=COLORS["text"], bbox=dict(boxstyle="round", facecolor="#d1fae5", alpha=0.9, pad=0.3))
        y -= 0.025
        for name, vmin, vmax, vinit, label, color in [
            ("kp_x", 0, 50, controller.kp_x, "Kp x", "#6ee7b7"),
            ("kd_x", 0, 50, controller.kd_x, "Kd x", "#6ee7b7"),
            ("ki_x", 0, 10, controller.ki_x, "Ki x", "#6ee7b7"),
        ]:
            ax_s = fig_anim.add_axes([SLIDER_LEFT, y - SH, SLIDER_W, SH])
            s = Slider(ax_s, label, vmin, vmax, valinit=vinit, color=color, valstep=(vmax - vmin) / 2000)
            _style_slider(s)
            sliders[name] = s
            y -= SS
    elif controller_type == "LQRController":
        fig_anim.text(RIGHT_X + RIGHT_W / 2, y, "State Cost Q", ha="center", fontsize=9, fontweight="bold",
                      color=COLORS["text"], bbox=dict(boxstyle="round", facecolor="#fef3c7", alpha=0.9, pad=0.3))
        y -= 0.025
        for name, vmin, vmax, vinit, label, color in [
            ("q_x", 0.1, 500, controller.Q[0, 0], "Q[0,0] Pos", "khaki"),
            ("q_x_dot", 0.1, 100, controller.Q[1, 1], "Q[1,1] Vel", "khaki"),
            ("q_theta", 0.1, 1000, controller.Q[2, 2], "Q[2,2] Ang", "khaki"),
            ("q_theta_dot", 0.1, 100, controller.Q[3, 3], "Q[3,3] AngVel", "khaki"),
        ]:
            ax_s = fig_anim.add_axes([SLIDER_LEFT, y - SH, SLIDER_W, SH])
            s = Slider(ax_s, label, vmin, vmax, valinit=vinit, color=color, valstep=(vmax - vmin) / 1000)
            _style_slider(s)
            sliders[name] = s
            y -= SS
        y -= 0.02
        fig_anim.text(RIGHT_X + RIGHT_W / 2, y, "Control Cost R", ha="center", fontsize=9, fontweight="bold",
                      color=COLORS["text"], bbox=dict(boxstyle="round", facecolor="#fecaca", alpha=0.9, pad=0.3))
        y -= 0.025
        ax_s = fig_anim.add_axes([SLIDER_LEFT, y - SH, SLIDER_W, SH])
        s = Slider(ax_s, "R", 0.001, 5, valinit=controller.R[0, 0], color="salmon", valstep=0.01)
        _style_slider(s)
        sliders["r"] = s
        y -= SS

    y -= 0.03
    # Buttons row at bottom of right panel (centered)
    total_btn_width = 4 * BW + 3 * BS
    btn_left = SLIDER_LEFT + (SLIDER_W - total_btn_width) / 2
    ax_play = fig_anim.add_axes([btn_left, y - BH, BW, BH])
    button_play = Button(ax_play, "Play", color=COLORS["accent_green"], hovercolor="#059669")
    ax_pause = fig_anim.add_axes([btn_left + BW + BS, y - BH, BW, BH])
    button_pause = Button(ax_pause, "Pause", color=COLORS["accent_amber"], hovercolor="#d97706")
    ax_reset = fig_anim.add_axes([btn_left + 2 * (BW + BS), y - BH, BW, BH])
    button_reset = Button(ax_reset, "Reset", color="#fecaca", hovercolor="#f87171")
    ax_apply = fig_anim.add_axes([btn_left + 3 * (BW + BS), y - BH, BW, BH])
    button_apply = Button(ax_apply, "Apply", color=COLORS["accent_blue"], hovercolor="#0284c7")
    
    # ============ WINDOW 2: Graphs ============
    fig_graphs = plt.figure(figsize=(14, 10), facecolor=COLORS["bg"])
    fig_graphs.canvas.manager.set_window_title("State & Control Graphs")

    # Create subplot grid: 2 rows x 2 columns
    ax_pos = fig_graphs.add_subplot(2, 2, 1)
    ax_angle = fig_graphs.add_subplot(2, 2, 2)
    ax_vel = fig_graphs.add_subplot(2, 2, 3)
    ax_control = fig_graphs.add_subplot(2, 2, 4)

    def _style_axes(ax):
        ax.set_facecolor("white")
        ax.tick_params(colors=COLORS["text"])
        ax.xaxis.label.set_color(COLORS["text"])
        ax.yaxis.label.set_color(COLORS["text"])
        ax.title.set_color(COLORS["text"])
        ax.grid(True, alpha=0.4, linestyle="--", color=COLORS["grid"])
        for spine in ax.spines.values():
            spine.set_color(COLORS["panel_dark"])

    # Position plot
    line_pos, = ax_pos.plot(time, x, color=COLORS["cart"], linewidth=2)
    ax_pos.set_xlabel("Time (s)", fontsize=9)
    ax_pos.set_ylabel("Position (m)", fontsize=9)
    ax_pos.set_title("Cart Position", fontsize=10, fontweight="bold")
    _style_axes(ax_pos)

    # Angle plot
    line_angle, = ax_angle.plot(time, np.degrees(theta), color=COLORS["pendulum"], linewidth=2)
    ax_angle.axhline(y=0, color=COLORS["text"], linestyle="--", alpha=0.4, linewidth=1)
    ax_angle.set_xlabel("Time (s)", fontsize=9)
    ax_angle.set_ylabel("Angle (°)", fontsize=9)
    ax_angle.set_title("Pendulum Angle", fontsize=10, fontweight="bold")
    _style_axes(ax_angle)

    # Velocities plot
    line_x_dot, = ax_vel.plot(time, x_dot, color=COLORS["cart"], label="Cart Velocity", alpha=0.9, linewidth=2)
    line_theta_dot, = ax_vel.plot(time, theta_dot, color=COLORS["pendulum"], label="Angular Velocity", alpha=0.9, linewidth=2)
    ax_vel.set_xlabel("Time (s)", fontsize=9)
    ax_vel.set_ylabel("Velocity", fontsize=9)
    ax_vel.set_title("Velocities", fontsize=10, fontweight="bold")
    ax_vel.legend(loc="upper right", fontsize=8)
    _style_axes(ax_vel)

    # Control input plot
    line_control, = ax_control.plot(time, control_input, color=COLORS["trace"], linewidth=2)
    ax_control.axhline(y=0, color=COLORS["text"], linestyle="--", alpha=0.4, linewidth=1)
    ax_control.set_xlabel("Time (s)", fontsize=9)
    ax_control.set_ylabel("Voltage (V)", fontsize=9)
    ax_control.set_title("Control Input", fontsize=10, fontweight="bold")
    _style_axes(ax_control)
    
    plt.tight_layout()
    
    # Animation state and shared buffers (only animation window is animated;
    # graphs window remains static vs. time for robustness across resizes)
    anim_state = {
        "paused": False,
        "frame": 0
    }
    
    trace_x, trace_y = [], []
    ARROW_DECAY_FRAMES = 500
    disturbance_memory = np.zeros(ARROW_DECAY_FRAMES)
    
    # Threading worker for simulation
    def simulation_worker():
        while sim_data["running"]:
            try:
                params = update_queue.get(timeout=0.1)
                if params is None:
                    break
                
                print("Running simulation with new parameters...")
                
                # Extract simulation parameters
                current_target_pos = params.get("target_pos", target_pos[0])
                current_noise_std = params.get("noise_std_dev", pendulum.noise_std_dev)
                current_disturbance = params.get("disturbance_level", pendulum.disturbance_level)
                
                # Update pendulum simulation parameters
                pendulum.noise_std_dev = current_noise_std
                pendulum.disturbance_level = current_disturbance
                
                # Update controller with new parameters (excluding simulation params)
                controller_params = {k: v for k, v in params.items() 
                                    if k not in ["target_pos", "noise_std_dev", "disturbance_level"]}
                
                if controller_type == "PIDController" or controller_type == "TrajectoryPIDController":
                    controller.set_kvalues(**controller_params)
                elif controller_type == "LQRController":
                    Q = np.diag([params["q_x"], params["q_x_dot"], params["q_theta"], params["q_theta_dot"]])
                    R = np.array([[params["r"]]])
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
                with sim_data["lock"]:
                    sim_data["trajectory"] = new_trajectory
                    sim_data["disturbance_log"] = new_disturbance_log
                    sim_data["control_log"] = np.array(new_control_log)
                    sim_data["update_ready"].set()
                
                print("✓ Simulation complete! Press Play to start.")
                
            except queue.Empty:
                continue
    
    # Start simulation worker thread
    sim_thread = threading.Thread(target=simulation_worker, daemon=True)
    sim_thread.start()
    
    def play_clicked(event):
        anim_state["paused"] = False
        print("▶ Playing")
    
    def pause_clicked(event):
        anim_state["paused"] = True
        print("⏸ Paused at t={:.2f}s".format(time[anim_state["frame"]]))
   
    def reset_clicked(event):
        """Reset playback to t=0 (instant). Use Apply to re-simulate with new params."""
        anim_state["paused"] = True
        anim_state["frame"] = 0
        trace_x.clear()
        trace_y.clear()
        disturbance_memory[:] = 0

        idx = 0
        cart.set_xy((x[idx] - cart_width/2, -cart_height/2))
        pend_x = x[idx] + pendulum.l * np.sin(-theta[idx])
        pend_y = pendulum.l * np.cos(-theta[idx])
        pendulum_line.set_data([x[idx], pend_x], [0, pend_y])
        trace.set_data([], [])
        time_text.set_text(f"Time: {time[idx]:.2f}s\nFrame: {idx}")
        disturbance_line.set_data([], [])
        disturbance_line.set_alpha(0.0)
        target_line.set_xdata([slider_target.val, slider_target.val])

        fig_anim.canvas.draw_idle()

    def apply_changes(event):
        anim_state["paused"] = True
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
        time_text.set_text("")
        disturbance_line.set_data([], [])
        disturbance_line.set_alpha(0.0)
        target_line.set_xdata([target_pos[0], target_pos[0]])
        return (cart, pendulum_line, trace, time_text, disturbance_line, target_line)

    def animate(i):
        nonlocal x, x_dot, theta, theta_dot, control_input
        
        # Check for simulation updates
        if sim_data["update_ready"].is_set():
            with sim_data["lock"]:
                trajectory = sim_data["trajectory"]
                disturbance_log = sim_data["disturbance_log"]
                new_control_log = sim_data["control_log"]
                sim_data["update_ready"].clear()
            
            # Update all data
            x[:] = trajectory[:, 0]
            x_dot[:] = trajectory[:, 1]
            theta[:] = trajectory[:, 2]
            theta_dot[:] = trajectory[:, 3]
            control_input[:] = (np.asarray(new_control_log).flatten() / 4) * 0.8

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
            anim_state["frame"] = 0
            anim_state["paused"] = True
            trace_x.clear()
            trace_y.clear()
            disturbance_memory[:] = 0

            # Update animation view to frame 0
            idx = 0
            cart.set_xy((x[idx] - cart_width/2, -cart_height/2))
            pend_x = x[idx] + pendulum.l * np.sin(-theta[idx])
            pend_y = pendulum.l * np.cos(-theta[idx])
            pendulum_line.set_data([x[idx], pend_x], [0, pend_y])
            trace.set_data([], [])
            time_text.set_text(f"Time: {time[idx]:.2f}s\nFrame: {idx}")
            disturbance_line.set_data([], [])
            disturbance_line.set_alpha(0.0)
            target_line.set_xdata([slider_target.val, slider_target.val])

        # Handle pause
        if anim_state["paused"]:
            target_line.set_xdata([slider_target.val, slider_target.val])
            return (cart, pendulum_line, trace, time_text, disturbance_line, target_line)
        
        # Update frame
        idx = anim_state["frame"] % len(x)
        anim_state["frame"] += 1
        
        # Update cart
        cart.set_xy((x[idx] - cart_width/2, -cart_height/2))
        
        # Update pendulum
        pend_x = x[idx] + pendulum.l * np.sin(-theta[idx])
        pend_y = pendulum.l * np.cos(-theta[idx])
        pendulum_line.set_data([x[idx], pend_x], [0, pend_y])
        
        # Update trace
        trace_x.append(pend_x)
        trace_y.append(pend_y)
        if len(trace_x) > 300:
            trace_x.pop(0)
            trace_y.pop(0)
        trace.set_data(trace_x, trace_y)
        
        # Update time
        time_text.set_text(f"Time: {time[idx]:.2f}s\nFrame: {idx}")

        # Update target position line to reflect slider
        target_line.set_xdata([slider_target.val, slider_target.val])
        
        # Disturbance visualization (simplified for perf)
        disturbance_memory[:-1] = disturbance_memory[1:]
        if sim_data["disturbance_log"] is not None and idx < len(sim_data["disturbance_log"]):
            Fd = sim_data["disturbance_log"][idx]
            disturbance_memory[-1] = float(np.asarray(Fd).sum()) if hasattr(Fd, "sum") else 0.0
        else:
            disturbance_memory[-1] = 0.0

        Fd_eff = disturbance_memory[-1]  # use latest only, skip decay logic
        if abs(Fd_eff) > 1e-6:
            arrow_length = 0.5 * np.tanh(abs(Fd_eff) / 3500)
            direction = np.sign(Fd_eff)
            x0, y0 = x[idx], 0.4
            x1, y1 = x0 + direction * arrow_length, y0
            disturbance_line.set_data([x0, x1], [y0, y1])
            disturbance_line.set_alpha(0.8)
        else:
            disturbance_line.set_alpha(0.0)
            disturbance_line.set_data([], [])
        
        return (cart, pendulum_line, trace, time_text, disturbance_line, target_line)

    # Create animation
    anim = animation.FuncAnimation(fig_anim, animate, init_func=init,
                                   frames=None, interval=25,  # 40 fps, smooth and responsive
                                   blit=True, repeat=True, cache_frame_data=False)
    
    def on_close(event):
        sim_data["running"] = False
        update_queue.put(None)
        sim_thread.join(timeout=1)
    
    fig_anim.canvas.mpl_connect("close_event", on_close)
    fig_graphs.canvas.mpl_connect("close_event", on_close)
    
    if save_gif:
        anim.save("pendulum_simulation.gif", writer="pillow", fps=int(1/pendulum.dt))
    
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
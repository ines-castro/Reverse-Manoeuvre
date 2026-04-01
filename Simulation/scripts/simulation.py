import numpy as np
import yaml
import time
import threading
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.widgets import Button, RadioButtons

from . import geometry_solver, control

class Simulation:
    """
    Real-time interactive simulation environment for reverse maneuver control.
    
    Provides visualization of robot trajectory, cart position, and telemetry data
    with a decoupled architecture running physics and MPC on a background thread.
    """

    def __init__(self, config_path):
        """
        Initialize the simulation environment.
        
        Args:
            config_path: Path to YAML configuration file
        """

        # -------- Configration for the simulation --------
        with open(config_path) as f:
            config = yaml.safe_load(f)

        # Initial parameters
        raw_state = config['physics']['initial_state']
        self.state = np.array([raw_state[0], raw_state[1], np.deg2rad(raw_state[2]), np.deg2rad(raw_state[3])])  # Convert angles to radians
        self.target = config['physics']['target']
        self.turning_radius = config['physics']['turning_radius']

        # Extract physics and cart specs from the dictionary
        cart_specs = config['cart_dimensions']
        self.cart_width = cart_specs['width']
        self.cart_length = cart_specs['length']

        # Initialize the other classes required for the simulation
        self.geometry_solver = geometry_solver.GeometrySolver(state=self.state, target=self.target, turning_radius=self.turning_radius)
        self.control = control.Controller(
            initial_state = self.state, 
            target = self.target, 
            geometry_solver = self.geometry_solver, 
            configs = config
        )

        # -------- State and history data --------
        self.cart_state = self.control.cart_model(self.state)
        self.x_history, self.y_history = [], []
        self.cart_history_patches = []
        self.cart_x_history, self.cart_y_history = [], []
        self.cross_error_history, self.hitch_error_history = [], []
        self.time_history = []
        self.dt = config['physics']['dt']
        self.current_time = 0.0
        self.current_vx, self.current_w = 0.0, 0.0

        # -------- UI Setup --------
        self.plot_design()
        self.add_widgets()
        self.update_text(0.0, 0.0)
        self.update_radio_choice('Control') 
        self.debug_plot()

        # -------- Decoupling of physics and visualization --------
        self.simulation_running = True
        self.lock = threading.Lock()

        # Physics runs in the background
        self.physics_thread = threading.Thread(target=self.physics_loop, daemon=True)
        self.physics_thread.start()

        # UI runs in the foreground
        self.ui_loop()

    def plot_design(self):
        """
        Configure the matplotlib figure layout and styling.
        """

        plt.ion()
        plt.style.use('dark_background')
        
        self.fig = plt.figure(figsize=(10, 6))
        gs = self.fig.add_gridspec(2, 2, width_ratios=[2, 1], height_ratios=[1, 1], 
                                   hspace=0.3, wspace=0.3, bottom=0.15)

        # ---- Big Plot: Trajectory ----
        self.ax_path = self.fig.add_subplot(gs[:, 0])
        self.ax_path.set_title("Path following trajectory", fontweight='bold')
        self.ax_path.set_xlabel("X [m]")
        self.ax_path.set_ylabel("Y [m]")
        self.ax_path.axis('equal')
        self.ax_path.set_aspect('equal')
        self.ax_path.grid(True, which='both', linewidth=0.5, color='gray')

        self.trajectory_line, = self.ax_path.plot([], [], color="#FFFFFF", linestyle='--', linewidth=1.2, label='Path')
        self.robot_marker, = self.ax_path.plot(self.state[0], self.state[1], marker='o', linestyle='', label='Robot')
        self.ax_path.plot(self.target[0], self.target[1], marker='d', linestyle='', label='Target')
        self.cart_center, = self.ax_path.plot(self.cart_state[0], self.cart_state[1], marker='s', linestyle='', label='Cart')

        # Heading arrow
        self.orientation_arrow = self.ax_path.quiver(
            self.state[0], self.state[1], 
            np.cos(self.state[2]), np.sin(self.state[2]), 
            angles='xy', scale_units='xy', scale=2,
            color='#FFA69E', width=0.01
        )
        self.cart = Rectangle(
            xy = (self.cart_state[0] - self.cart_width / 2, self.cart_state[1] - self.cart_length / 2),
            width = self.cart_width,
            height = self.cart_length,
            edgecolor='cyan',
            facecolor='none',
            linewidth=1,
            angle=np.rad2deg(self.cart_state[2]),
            rotation_point='center'
        )
        self.ax_path.add_patch(self.cart)
        self.cart_trajectory_line, = self.ax_path.plot([], [], color = "cyan", linestyle= '--',linewidth=1.2, label='Cart Path')

        # ---- Mini Top Plot: Cross Track Error ----
        self.ax_cross_error = self.fig.add_subplot(gs[0, 1])
        self.ax_cross_error.set_title("Cross Track Error", fontweight='bold')
        self.ax_cross_error.set_xlabel("Time [s]")
        self.ax_cross_error.set_ylabel("Error [m]")
        self.ax_cross_error.grid(True, which='both', linewidth=0.5, color='gray')
        self.line_cross, = self.ax_cross_error.plot([], [], color="#98FBCB", linestyle='--', linewidth=1.2, label='Cross Error')

        # ---- Mini Bottom Plot: Hitch Angle ----
        self.ax_hitch = self.fig.add_subplot(gs[1, 1])
        self.ax_hitch.set_title("Hitch Angle", fontweight='bold')
        self.ax_hitch.set_xlabel("Time [s]")
        self.ax_hitch.set_ylabel("Angle [deg]")
        self.ax_hitch.grid(True, which='both', linewidth=0.5, color='gray')
        self.line_hitch, = self.ax_hitch.plot([], [], color="#FFEE8C", linestyle='--', linewidth=1.2, label='Hitch Angle')

    def add_widgets(self):
        """
        Add interactive UI widgets (stop button and mode toggle).
        """
        # Color palette
        ACCENT = '#FFA69E'
        WHITE = '#F2F2F2'
        RED_NORMAL = '#3D1A1A'
        RED_HOVER = '#C0392B'

        # ---- STOP BUTTON ----
        ax_stop = self.fig.add_axes([0.1, 0.05, 0.08, 0.04])
        self.bstop = Button(
            ax_stop, 
            'STOP', 
            color=RED_NORMAL, 
            hovercolor=RED_HOVER
        )
        self.bstop.label.set_color(WHITE)
        self.bstop.label.set_fontweight('bold')
        self.bstop.label.set_fontsize(9)
        for spine in ax_stop.spines.values():
            spine.set_visible(False)
        self.bstop.on_clicked(self.stop_simulation)

        # ---- RADIO BUTTONS ----
        rax = self.fig.add_axes([0.22, 0.035, 0.12, 0.07])
        rax.set_facecolor(self.fig.get_facecolor())
        for spine in rax.spines.values():
            spine.set_visible(False)
            
        self.radio_button = RadioButtons(rax, labels=('Control', 'Debug'), active=0, activecolor=ACCENT)
        
        for label in self.radio_button.labels:
            label.set_color(WHITE)
            label.set_fontsize(10)
            label.set_fontfamily('sans-serif')
        self.radio_button.on_clicked(self.update_radio_choice)

        # ---- Velocity Display ----
        pos = self.ax_path.get_position()
        self.velocity_text = self.ax_path.text(
            pos.x0 - 0.05, pos.y1 + 0.05, 
            'Linear V: 0.00 m/s\nAngular W: 0.00 rad/s', 
            transform=self.ax_path.transAxes,
            verticalalignment='top',
            color=ACCENT, 
            fontsize=10,
            fontweight='bold',
            fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.8', facecolor='#262626', 
                     edgecolor='#404040', alpha=0.9, linewidth=1)
        )
    def update_text(self, vx, w):
        text_str = f'Linear V: {vx:.2f} m/s\nAngular W: {w:.2f} rad/s'
        self.velocity_text.set_text(text_str)

    def stop_simulation(self, event):
        """Stop the simulation when the stop button is clicked."""
        self.simulation_running = False

    def update_radio_choice(self, label):
        """Toggle between Control and Debug visualization modes."""
        if label == 'Control':
            self.debug_mode = False
            # Hide the sliders, the velocities come from control algorithms
            self.ax_cross_error.set_visible(False)
            self.ax_hitch.set_visible(False)

        elif label == 'Debug':
            self.debug_mode = True
            self.ax_cross_error.set_visible(True)
            self.ax_hitch.set_visible(True)

        self.fig.canvas.draw_idle()

    def debug_plot(self):
        """Visualize path geometry for debugging purposes."""
        graphic_helpers, control_helpers, overshoot_case = self.geometry_solver.calculate_path_geometry()

        if overshoot_case:
            # The robot is after the entry point
            x_range = np.array([self.target[0] - graphic_helpers['offset'], self.target[0] - graphic_helpers['offset']])
            y_range = np.array([-2,  self.state[1] + 2])
            self.ax_path.plot(x_range, y_range, color='#837ACF', linestyle=':', alpha=0.6, label='Heading Line')

            # Reflected circle
            self.ax_path.plot(graphic_helpers['circle2_center'][0], graphic_helpers['circle2_center'][1], marker = 'x', color='#837ACF')
            circle = plt.Circle(graphic_helpers['circle2_center'], graphic_helpers['circle_radius'], color='#837ACF', fill=False, linestyle='--', alpha=0.6, label='Turning Circle')
            self.ax_path.add_patch(circle)

        self.ax_path.plot(control_helpers['turning_point'][0], control_helpers['turning_point'][1], marker = 'x', color='#e9f6d4', label='Turning Point')

        # Maker for robot and tangent points
        self.ax_path.plot(graphic_helpers['vertex'][0], graphic_helpers['vertex'][1], marker = '*', color='white', label='Vertex')
        self.ax_path.plot(control_helpers['exit_point'][0], control_helpers['exit_point'][1], marker = 'x', color='#e9f6d4', label='Exit Point')

        # Line for the robot's heading
        x_range = np.array([-2, self.state[0] + 2])
        y_range = graphic_helpers['heading_line'][0] * x_range + graphic_helpers['heading_line'][1]
        self.ax_path.plot(x_range, y_range, "#B1AEAE", linestyle=':', alpha=0.6, label='Heading Line')

        # Line for the target's vertical line
        x_range = np.array([self.target[0], self.target[0]])
        y_range = np.array([-2, self.state[1] + 2])
        self.ax_path.plot(x_range, y_range, "#B1AEAE", linestyle=':', alpha=0.6, label='Heading Line')

        # Circle for the turning radius
        self.ax_path.plot(graphic_helpers['circle_center'][0], graphic_helpers['circle_center'][1], marker = 'x', color='#837ACF')
        circle = plt.Circle(graphic_helpers['circle_center'], graphic_helpers['circle_radius'], color='#837ACF', fill=False, linestyle='--', alpha=0.6, label='Turning Circle')
        self.ax_path.add_patch(circle)

    # ---------------------------------------------------------
    # ENGINES
    # ---------------------------------------------------------
    def physics_loop(self):
        """High-frequency background thread for physics and MPC calculations."""

        while self.simulation_running:
            start_tick = time.time()
            new_state, new_cart_state, vx, w, e_cross, finished = \
               self.control.path_following(self.state, self.target)
            
            # Update shared state with thread-safe locking
            with self.lock:
                self.state = new_state
                self.cart_state = new_cart_state
                self.current_vx = vx
                self.current_w = w

                # Update histories                
                self.current_time += self.dt
                self.time_history.append(self.current_time)
                self.x_history.append(self.state[0])
                self.y_history.append(self.state[1])
                self.cart_x_history.append(self.cart_state[0])
                self.cart_y_history.append(self.cart_state[1])
                self.cross_error_history.append(e_cross)
                self.hitch_error_history.append(np.rad2deg(self.state[3]))

                if finished:
                   self.simulation_running = False
                   print("--- Physics engine shutting down. ---")
                   break
            
            # Maintain fixed time step
            elapsed = time.time() - start_tick
            time.sleep(max(0, self.dt - elapsed))

    def ui_loop(self):
        """Main rendering loop for visualization updates."""
        self.plot_state()

        while plt.fignum_exists(self.fig.number):
            with self.lock:
                # Update the plot with the latest state
                self.plot_state()
                self.update_text(self.current_vx, self.current_w)

                if self.debug_mode: 
                    self.plot_cross_track_error()
                    self.plot_hitch_angle()

            # UI refresh rate
            plt.pause(0.01)

        self.simulation_running = False
        print("Figure closed, stopping simulation.")

    # ---------------------------------------------------------
    # MOVEMENT UPDATE
    # ---------------------------------------------------------
    def plot_state(self):
        """Update robot and cart visualization."""

        self.trajectory_line.set_data(self.x_history, self.y_history)

        # Update robot state
        self.robot_marker.set_data([self.state[0]], [self.state[1]])
        self.orientation_arrow.set_offsets([self.state[0], self.state[1]])
        self.orientation_arrow.set_UVC(np.cos(self.state[2]), np.sin(self.state[2]))
        self.fig.canvas.draw()

        # Update cart position
        self.cart_center.set_data([self.cart_state[0]], [self.cart_state[1]])
        self.cart.set_xy((self.cart_state[0] - self.cart_width / 2, self.cart_state[1] - self.cart_length / 2))
        self.cart.angle = np.rad2deg(self.cart_state[2])

        # Add ghosting effect every 10 frames for performance
        if len(self.cart_x_history) % 10 == 0:  
            rect = Rectangle(
                xy=(self.cart_state[0] - self.cart_width / 2,
                    self.cart_state[1] - self.cart_length / 2),
                width=self.cart_width,
                height=self.cart_length,
                edgecolor='cyan',
                facecolor='none',
                linewidth=0.8,
                alpha=0.3,
                angle=np.rad2deg(self.cart_state[2]),
                rotation_point='center'
            )
            self.ax_path.add_patch(rect)

        self.ax_path.relim()
        self.ax_path.autoscale_view()
        self.ax_path.set_aspect('equal')

    def plot_cross_track_error(self):
        """Update cross-track error plot."""
        self.line_cross.set_data(self.time_history, self.cross_error_history)
        self.fig.canvas.draw()
        self.ax_cross_error.relim()
        self.ax_cross_error.autoscale_view()

    def plot_hitch_angle(self):
        """Update hitch angle plot."""
        self.line_hitch.set_data(self.time_history, self.hitch_error_history)
        self.fig.canvas.draw()
        self.ax_hitch.relim()
        self.ax_hitch.autoscale_view()

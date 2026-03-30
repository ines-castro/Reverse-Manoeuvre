import numpy as np
import pandas as pd
import yaml
import scipy
import geometry_solver
from scipy.optimize import minimize

class Controller:

    def __init__(self, initial_state, target, geometry_solver, configs: dict):
        
        self.dt = configs['physics']['dt']

        # Load controller tunning parameters
        controller_config = configs['controller']
        self.constant_vx = controller_config['constant_vx']
        self.K_dist = controller_config['K_dist']
        self.K_turn = controller_config['K_turn']
        self.K_hitch = controller_config['K_hitch']
        self.lookahead_distance = controller_config['lookahead_distance']
        self.horizon = controller_config['horizon']

        # Load cart dimensions
        cart_specs = configs['cart_dimensions']
        self.cart_width = cart_specs['width']
        self.fixed_wheel_dist = cart_specs['fixed_wheel_dist']
        self.gripper_length = cart_specs['gripper_length']
        self.gripper_angle_limit = np.deg2rad(cart_specs['gripper_angle_limit'])

        # Total distance between the robot's center and the cart's fixed wheels
        self.cart_wheelbase = self.gripper_length + self.fixed_wheel_dist  

        # Calculate the path guidelines
        graphic_helpers, control_helpers, overshoot_case = geometry_solver.calculate_path_geometry()

        # Unpack the necessary variables
        waypoints = {
            'initial': initial_state[:2],
            'turning': control_helpers['turning_point'],
            'exit': control_helpers['exit_point'],
            'target': np.array(target)
        }

        # Defined the path by a discrete set of points
        path_points = []
        circles_centers = [graphic_helpers['circle_center'], graphic_helpers['circle2_center']]
        for s in np.linspace(0, 8.3, 500):
            pt = geometry_solver.path(s, waypoints, circles_centers, graphic_helpers['circle_radius'], overshoot_case)
            path_points.append([pt[0], pt[1]])

        # Save to csv for debugging purposes
        df = pd.DataFrame(path_points, columns=['x', 'y'])
        df.to_csv('coco.csv', index=False)

        # Use it as reference for the path planning
        self.path = np.array(path_points)

        self.last_w = 0.0  # Store the last angular velocity for smoothing
        self.prev_hitch_error = 0.0
    
    def robot_model(self, state, inputs):
        '''
        Simple kinematic model of the cart
        Args:
            state: State of the robot [x, y, theta, hitch_angle]
            inputs: Control inputs [vx, w]
        '''
        x, y, theta, gamma = state
        vx, w = inputs

        dx = vx * np.cos(theta)
        dy = vx * np.sin(theta)
        dtheta = w

        # Cart Kinematics
        w_cart = (vx / self.cart_wheelbase) * np.sin(gamma)
        
        d_hitch = - w_cart - w

        x_next = x + dx * self.dt
        y_next = y + dy * self.dt
        theta_next = theta + dtheta * self.dt
        gamma_next = gamma + d_hitch * self.dt

        # Mechanical limit of gripper
        gamma_next = np.clip(gamma_next, -self.gripper_angle_limit, self.gripper_angle_limit)

        state_next = [x_next, y_next, theta_next, gamma_next]

        return state_next
    
    def cart_model(self, state):
        '''
        Calculate the position and heading of the cart based on the robot's state
        '''
        # Hitch point at the back of the robot
        hitch_x = state[0] - self.gripper_length * np.cos(state[2])
        hitch_y = state[1] - self.gripper_length * np.sin(state[2])

        cart_heading = state[2] + state[3] # hitch angle = cart heading - robot heading
        cart_x = hitch_x - self.fixed_wheel_dist * np.cos(cart_heading)
        cart_y = hitch_y - self.fixed_wheel_dist * np.sin(cart_heading)
        
        cart_state = [cart_x, cart_y, cart_heading]

        return cart_state
        
    def cost(self, state, w_seq, closest_index, horizon):
        '''
        Drives the optimisation process by quantifying the error between the current state
        and the the desired trajectory.
        '''
        current_state = np.array(state).copy()

        # Avoid crashing when near the end of the path
        end_index = closest_index + horizon 
        if end_index <= len(self.path):
            ref_path = self.path[closest_index:end_index]   
        else: 
            pad_length = end_index - len(self.path)
            padding = np.repeat(self.path[-1][None, :], pad_length, axis=0)
            ref_path = np.vstack((self.path[closest_index:], padding))

        total_cost = 0.0
        for i in range(horizon):

            # Optimise only the angular velocity
            w = w_seq[i]
            inputs = [self.constant_vx, w] 

            # Calculate the cart state at the next time step
            state_i = self.robot_model(current_state, inputs)
            cart_state_i = self.cart_model(state_i)
            
            # Distance from the cart state to the reference path point
            distance = (cart_state_i[0] - ref_path[i][0]) ** 2 + (cart_state_i[1] - ref_path[i][1]) ** 2
            
            # Steering effort cost to avoid agressive turns
            turning = inputs[1] ** 2

            # Hitch angle cost to avoid reaching the hitting mechanical limit
            hitch = (np.exp(abs(state_i[3]) / self.gripper_angle_limit * 5) - 1)

            if i == 0:
                delta_w = (w_seq[i] - self.last_w)**2
            else:
                delta_w = (w_seq[i] - w_seq[i-1])**2


            total_cost += self.K_dist * distance + self.K_turn * delta_w + self.K_hitch * hitch

            dist_to_end = np.linalg.norm(cart_state_i[:2] - self.path[-1])
            if dist_to_end < 2.0:
                # This multiplier grows exponentially as dist_to_end -> 0
                finish_logic_weight = np.exp(2.0 - dist_to_end) 
                total_cost += finish_logic_weight * (state_i[3]**2)

            current_state = state_i

        return total_cost

    def hitch_constraint(self, u, state):
        """
        Ensures the predicted hitch angle never exceeds its mechanical limit.
        Returns >=0 if feasible.
        """
        current_state = np.array(state).copy()
        min_margin = float('inf')
        for w in u:
            current_state = self.robot_model(current_state, [self.constant_vx, w])
            margin = self.gripper_angle_limit - abs(current_state[3])
            min_margin = min(min_margin, margin)
        return min_margin  # >= 0 means all steps are feasible

    def path_following(self, state, target):
        '''
        Path planning mode where the robot calculates the velocities needed to reach the target
        '''
        # Cart current state
        cart_state = self.cart_model(state)

        # ---- PURE PURSUIT ----

        # Calculate the distances between each way point and robot position
        distance_set = []
        for i in range(len(self.path)):
            path_point = self.path[i]
            dist = np.linalg.norm(path_point - cart_state[:2])
            distance_set.append(dist)

        # Obtain the point which has the minimum distance to the robot position
        min_index = np.argmin(distance_set)
        closest_point = self.path[min_index]

        # Local target way point
        local_target = closest_point
        # Search for the point in the path that is at least lookahead_distance away from the closest point
        for i in range(min_index, len(self.path)):
            dist = np.linalg.norm(self.path[i] - closest_point)
            if dist >= self.lookahead_distance:
                local_target = self.path[i]
                break

        # ---- MODEL PREDICTIVE CONTROL ----
        #w_guess = np.full(self.horizon, self.last_w)
        w_guess = np.zeros(self.horizon)
        #  Bounds for the control inputs
        bounds = [(-np.deg2rad(45), np.deg2rad(45))] * self.horizon
        constraint = {
            'type': 'ineq',  # inequality: must be >=0
            'fun': lambda u: self.hitch_constraint(u, state)
        }

        try:
            res = minimize(
                lambda u: self.cost(state, u, min_index, self.horizon),
                w_guess,
                method='SLSQP',
                bounds=bounds,
                
                options=dict(maxiter=100, disp=False)  # ← add disp=False
            )
        except Exception as e:
            print(f"Optimization error: {e}")
            res = type('obj', (object,), {'success': False, 'x': w_guess})()

        vx = self.constant_vx 
        if res.success:
            w = res.x[0]  # Optimal angular velocity for the first time step     
        else:
            w = -0.1
        self.last_w = w  

        # Calculate the cross track error
        e_cross = np.linalg.norm(closest_point - cart_state[:2])

        # Stop condition
        finished = False
        if np.linalg.norm(np.array(cart_state[:2]) - np.array(target)) < 0.1:
            vx = 0.0
            w = 0.0
            print("Target reached!")
            finished = True

        # Update State
        state = self.robot_model(state, [vx, w])
        cart_state = self.cart_model(state)

        return state, cart_state, vx, w, e_cross, finished

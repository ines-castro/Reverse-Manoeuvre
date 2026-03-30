import math
import numpy as np

class GeometrySolver:

    def __init__(self, state, target, turning_radius):
        self.debug = True
        self.state = state
        self.target = target
        self.r = turning_radius

    def calculate_path_geometry(self):
        '''
        Calculate the geometry of the path in all stages of reverse maneuver
        '''
        # Heading unit vector
        vec_h = np.array([np.cos(self.state[2]), np.sin(self.state[2])])
        vec_prep = np.array([-np.sin(self.state[2]), np.cos(self.state[2])])  
        
        m_heading = np.tan(self.state[2])
        b_heading = self.state[1] - m_heading * self.state[0]

        # -------- Mandatory turning circle --------
        circle_radius = self.r
        vertex, circle_center, _ = self.get_tangent_circle(m_heading, b_heading, float('inf'), self.target[0], self.state[:2], self.target, circle_radius)

        print(">>>>>>>>>>>>>>>>>>>>>")

        # Calculate both tangent points to the circle
        tangent_heading, _ = self.get_intersection_line_circle(m_heading, b_heading, circle_center, circle_radius)
        tangent_target, _ = self.get_intersection_line_circle(float('inf'), self.target[0], circle_center, circle_radius)
        tangent_points = [tangent_heading, tangent_target]

        # -------- Overshoot logic --------
        heading_vector = np.array([np.cos(self.state[2]), np.sin(self.state[2])])
        offset_vector = self.state[:2] - tangent_points[0]
       
        if np.dot(heading_vector, offset_vector) < 0:

            print("The circle was shifted to adjust for the robot's position after the entry point.")
            overshoot_case = True

            # We shift the center by the distance to entry point 
            circle_center = circle_center + offset_vector
            offset = np.linalg.norm(offset_vector)

            # Derive the center of the second triangle geometrically
            x_c = self.target[0] - circle_radius
            dx = (x_c - circle_center[0])**2
            dy = np.sqrt(max(0, (2 * circle_radius)**2 - dx))  # Pitagoras theorem

            y_c = circle_center[1] - dy

            # Second circle with same radius as the first one
            circle2_center = np.array([x_c, y_c])

            # Used as a turning point
            circles_intersection = (circle2_center + circle_center) / 2

            exit_point = self.get_intersection_line_circle(float('inf'), self.target[0], circle2_center, circle_radius)[0]

        else: 
            # The control points are the ones tangent to the first circle
            entry_point = tangent_points[0]
            exit_point = tangent_points[1]
            # We keep the circle center as initially calculated
            overshoot_case = False
            circle2_radius = None
            circle2_center = None
            circles_intersection = None

        # ---- Graphic helpers ----
        graphic_helpers = {
            'heading_line': (m_heading, b_heading),
            'vertex': vertex,
            'circle_center': circle_center,
            'circle2_center': circle2_center ,
            'circle_radius': circle_radius,
            'offset': offset if overshoot_case else None,
        }

        control_helpers = {
            'turning_point': circles_intersection if overshoot_case else entry_point,
            'exit_point': exit_point,
        }

        return graphic_helpers, control_helpers, overshoot_case

    def path(self, s: float, waypoints: dict, centers: list, radius: float, overshoot_case: bool) -> np.array:
        '''
        Parameterize the path with respect to the distance traveled along.
        Args: 
            s: total distance traveled along the path
            waypoints: points that define the path 
                - initial: where the robot starts (if overshoot case, it is the point where the robot starts turning)
                - turning: where the robot changes circle or starts turning if no overshoot
                - exit: where the robot stops turning and heads straight to the target 
                - target: where the robot wants to go
            centers : list of centers of the turning circles (2 if overshoot case, 1 otherwise)
        
        Returns:
            The point on the path corresponding to the distance s.

        '''

        if overshoot_case:
            distance_to_turn = 0.0 

            # First arc angles
            phi_start = np.arctan2(waypoints['initial'][1] - centers[0][1], waypoints['initial'][0] - centers[0][0])
            phi_end = np.arctan2(waypoints['turning'][1] - centers[0][1], waypoints['turning'][0] - centers[0][0])
                
            # Second arc angles
            phi_start2 = np.arctan2(waypoints['turning'][1] - centers[1][1], waypoints['turning'][0] - centers[1][0])
            phi_end2 = np.arctan2(waypoints['exit'][1] - centers[1][1], waypoints['exit'][0] - centers[1][0])
            arc_length2 = radius * abs(self.angle_diff(phi_start2, phi_end2))
        else:
            distance_to_turn = np.linalg.norm(waypoints['initial'] - waypoints['turning'])
            
            phi_start = np.arctan2(waypoints['turning'][1] - centers[0][1], waypoints['turning'][0] - centers[0][0])
            phi_end = np.arctan2(waypoints['exit'][1] - centers[0][1], waypoints['exit'][0] - centers[0][0])
            
            # There is no second arc
            arc_length2 = 0.0
       
        # First arc length
        arc_length = radius * abs(self.angle_diff(phi_start, phi_end))

        # Final straight line
        distance_to_target = np.linalg.norm(waypoints['target'] - waypoints['exit'])

        # Calculate total path length
        path_length = distance_to_turn + arc_length + arc_length2 + distance_to_target

        # If s is larger than the path, just stay at the target
        s = max(0, min(s, path_length))

        # ---- Segment 1: Initial straight line ----
        if 0 <= s < distance_to_turn:
            v = (waypoints['turning'] - waypoints['initial']) / distance_to_turn
            x = waypoints['initial'][0] + v[0] * s
            y = waypoints['initial'][1] + v[1] * s

        # ---- Segment 2: First arc ----
        elif distance_to_turn <= s < (distance_to_turn + arc_length):
            s_arc = s - distance_to_turn

            current_phi = phi_start + (s_arc / radius) * np.sign(self.angle_diff(phi_start, phi_end))
            
            c1 = centers[0]
            x = c1[0] + radius * np.cos(current_phi)
            y = c1[1] + radius * np.sin(current_phi)

        # ---- Segment 3: Second arc (only in the overshoot case) ----
        elif arc_length2 > 0 and (distance_to_turn + arc_length) <= s <= (distance_to_turn + arc_length + arc_length2):
            s_arc2 = s - distance_to_turn - arc_length

            current_phi = phi_start2 + (s_arc2 / radius) * np.sign(self.angle_diff(phi_start2, phi_end2))
            c2 = centers[1]
            x = c2[0] + radius * np.cos(current_phi)
            y = c2[1] + radius * np.sin(current_phi)

        elif (distance_to_turn + arc_length + arc_length2) <= s < path_length:
            s_final = s - distance_to_turn - arc_length - arc_length2

            v = (waypoints['target'] - waypoints['exit']) / distance_to_target
            x = waypoints['exit'][0] + v[0] * s_final
            y = waypoints['exit'][1] + v[1] * s_final

        else: 
            x,y = waypoints['target']

        return np.array([x, y])

    # ---------------------------------------------------------
    # AUXILIARY FUNCTIONS
    # ---------------------------------------------------------
    def angle_diff(self, a, b):
        '''
        Calculates the difference between two angles.
        Normalizes it to be between -pi and pi.
        '''
        d = b - a
        return (d + np.pi) % (2 * np.pi) - np.pi

    def get_tangent_circle(self, m1, b1, m2, b2, A, B, radius = None):
        '''
        Calculate the center of a circle that is tangent to two lines 
        and which direction is defined by A and B.
         - Line 1: y = m1 * x + b1
         - Line 2: y = m2 * x + b2
        '''

        # Intersection 
        # Intersection between the two lines
        vertex = self.get_intersection_lines(m1, b1, m2, b2)

        # Direction vector for the lines
        vec_1 = np.array(A) - vertex
        vec_2 = np.array(B) - vertex
       
        bisector = vec_1 / np.linalg.norm(vec_1) + vec_2 / np.linalg.norm(vec_2)
        bisector_unit = bisector / np.linalg.norm(bisector)
        vec_1_unit = vec_1 / np.linalg.norm(vec_1)
        vec_2_unit = vec_2 / np.linalg.norm(vec_2)

        # Calculate the angle between bisector and line
        dot_product = np.clip(np.dot(bisector_unit, vec_1_unit), -1.0, 1.0)
        half_angle = np.arccos(dot_product)

        if radius is None:
            # If radius is not provided, we assume it is supposed to touch A and B
            h = np.linalg.norm(vertex - A)
            circle_radius = np.tan(half_angle) * h
        else:
            circle_radius = radius

        # Calculate the distance from vertex to center (Hypotenuse)
        dist_to_center = circle_radius / np.sin(half_angle)

        # Place the center by moving from the vertex along the bisector
        circle_center = vertex + dist_to_center * (bisector / np.linalg.norm(bisector))

        return vertex, circle_center, circle_radius
    
    def get_intersection_lines(self, m1, b1, m2, b2):
        '''
        Calculates the intersection point between two lines, given the slopes and intercepts of the lines.
         - Line 1: y = m1 * x + b1
         - Line 2: y = m2 * x + b2
        '''
        # Both lines are vertical and parallel, no intersection
        if m1 == float('inf') and m2 == float('inf'):
            intersection = [float('inf'), float('inf')]
        
        # Line 1 is vertical: x = b1
        elif m1 == float('inf'):
            intersection = [b1, m2 * b1 + b2]
        
        elif m2 == float('inf'):
            intersection = [b2, m1 * b2 + b1]
        
        else: 
            # In matrix system form: A * [x, y] = B
            A = np.array([[-m1, 1], [-m2, 1]])
            B = np.array([b1, b2])

            if np.linalg.det(A) == 0:
                print("Lines are parallel, no intersection.")
                intersection = [float('inf'), float('inf')]
            else:
                intersection = np.linalg.solve(A, B)
        
        return intersection
    
    def get_intersection_line_circle(self, m, b, center, radius):
        '''
        Calculates the intersection points between a line and a circle.
         - Line: 
            normal: y = m * x + b
            horizontal: m = 0 => y = b
            vertical: m = inf => x = b
         - Circle: (x - cx)^2 + (y - cy)^2 = r^2
        '''
        if m == float('inf'):
            # Vertical line case: x = b
            x = b
            dx = abs(x - center[0])

            # Handle floating point precision issues
            if abs(dx - radius) < 1e-9:
                dx = radius

            # Case 1: No intersection points
            if dx > radius:
                return None, None
            
            # Case 2: One intersection point (tangent)
            elif dx == radius:
                return [b, center[1]], None
            
            # Case 3: Two intersection points
            else:
                # Pytagoras theorem to find the y distance
                dy = np.sqrt(radius**2 - dx**2)
                return [b, center[1] + dy], [b, center[1] - dy]  

        # Normal or horizontal line case
        A = 1 + m**2    
        B = 2 * (m * b - m * center[1] - center[0])
        C = center[0]**2 + center[1]**2  + b**2 - 2 * b * center[1] - radius**2

        discriminant = B**2 - 4 * A * C

        # Handle floating point precision issues
        if abs(discriminant) < 1e-9:
            discriminant = 0.0

        if discriminant < 0:
            print("No intersection between line and circle.")
            return [float('inf'), float('inf')]

        sqrt_disc = np.sqrt(discriminant)
        x1 = (-B + sqrt_disc) / (2 * A)
        x2 = (-B - sqrt_disc) / (2 * A)

        if x1 == x2:
            print("Line is tangent to the circle, one intersection point.")
            y1 = m * x1 + b
            return [x1, y1], None
        else:
            y1 = m * x1 + b
            y2 = m * x2 + b
            return [x1, y1], [x2, y2]
        
    def get_reflected_point(self, point, m, b):
        '''
        Reflects the point across the line defined by m and b.
         - Line:
            normal: y = m * x + b
            horizontal: m = 0 => y = b
            vertical: m = inf => x = b
        '''
        
        if m == float('inf'):
            # Vertical line case: x = b
            reflected_point = np.array([2 * b - point[0], point[1]])
        elif m == 0:
            # Horizontal line case: y = b
            reflected_point = np.array([point[0], 2 * b - point[1]])
        else:
            # Perpendicular to the line: y = -1/m * x + c
            c = point[1] + (1/m) * point[0]
            # Intersection point between the line and the perpendicular
            intersection = self.get_intersection_lines(m, b, -1/m, c)

            # The intersection point is the midpoint between the original point and the reflected point
            reflected_x = 2 * intersection[0] - point[0]
            reflected_y = 2 * intersection[1] - point[1]

            reflected_point = np.array([reflected_x, reflected_y])

        return reflected_point
    

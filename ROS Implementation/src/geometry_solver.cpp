#include "geometry_solver.h"
#include <cmath>
#include <algorithm>
#include <iostream>

GeometrySolver::GeometrySolver(const RobotState& state, const Point2D& target, double turning_radius)
    : m_state(state)
    , m_target(target)
    , m_turning_radius(turning_radius)
    , m_debug(true)
    , m_overshoot_case(false)
    , m_geometry_calculated(false)
{
}

bool GeometrySolver::calculatePathGeometry()
{
    double m_heading = std::tan(m_state.theta);
    double b_heading = m_state.y - m_heading * m_state.x;
    
    // Determine target line orientation based on displacement to target
    double dx = std::abs(m_target.x - m_state.x);
    double dy = std::abs(m_target.y - m_state.y);
    bool use_vertical_target = (dy > dx);  // More vertical displacement → vertical target line
    
    double m_target_line, b_target_line;
    if (use_vertical_target) {
        // Vertical line at target x-coordinate
        m_target_line = INF;
        b_target_line = m_target.x;
        if (m_debug) std::cout << "Using VERTICAL target line at x = " << m_target.x << " (Δx=" << dx << ", Δy=" << dy << ")" << std::endl;
    } else {
        // Horizontal line at target y-coordinate
        m_target_line = 0.0;
        b_target_line = m_target.y;
        if (m_debug) std::cout << "Using HORIZONTAL target line at y = " << m_target.y << " (Δx=" << dx << ", Δy=" << dy << ")" << std::endl;
    }
    
    // -------- Mandatory turning circle --------
    double circle_radius = m_turning_radius;
    Point2D vertex, circle_center;
    double calc_radius;
    
    if (!getTangentCircle(m_heading, b_heading, m_target_line, b_target_line,
                         m_state.position(), m_target,
                         vertex, circle_center, calc_radius, circle_radius))
    {
        if (m_debug) std::cout << "Failed to calculate tangent circle" << std::endl;
        return false;
    }

    std::cout << "Calculated circle center: (" << circle_center.x << ", " << circle_center.y << ") radius: " << calc_radius << std::endl;
    
    // Calculate tangent points to the circle
    Point2D tangent_heading_1, tangent_heading_2;
    Point2D tangent_target_1, tangent_target_2;
    
    int num_tangents_heading = getIntersectionLineCircle(m_heading, b_heading, 
                                                         circle_center, circle_radius,
                                                         tangent_heading_1, tangent_heading_2);
    
    int num_tangents_target = getIntersectionLineCircle(m_target_line, b_target_line,
                                                        circle_center, circle_radius,
                                                        tangent_target_1, tangent_target_2);
    
    if (num_tangents_heading < 1 || num_tangents_target < 1) {
        if (m_debug) std::cout << "Failed to find tangent points" << std::endl;
        return false;
    }
    
    // Select correct tangent points for REVERSE motion
    Point2D tangent_heading, tangent_target;
    Point2D heading_dir(std::cos(m_state.theta), std::sin(m_state.theta));
    
    if (num_tangents_heading == 2) {
        // For REVERSE: pick the tangent point BEHIND the robot
        Point2D to_tangent_1 = tangent_heading_1 - m_state.position();
        Point2D to_tangent_2 = tangent_heading_2 - m_state.position();
        
        // Dot product: negative = behind (reverse direction)
        double dot1 = heading_dir.dot(to_tangent_1);
        double dot2 = heading_dir.dot(to_tangent_2);
        
        if (m_debug) {
            std::cout << "Heading tangent selection:" << std::endl;
            std::cout << "  T1: (" << tangent_heading_1.x << ", " << tangent_heading_1.y << ") dot=" << dot1 << std::endl;
            std::cout << "  T2: (" << tangent_heading_2.x << ", " << tangent_heading_2.y << ") dot=" << dot2 << std::endl;
        }
        
        // Pick the one MORE behind (more negative dot product)
        tangent_heading = (dot1 < dot2) ? tangent_heading_1 : tangent_heading_2;
        
        if (m_debug) {
            std::cout << "  Chose: (" << tangent_heading.x << ", " << tangent_heading.y << ")" << std::endl;
        }
    } else {
        tangent_heading = tangent_heading_1;
    }
    
    if (num_tangents_target == 2) {

        // For target tangent: pick the one that's on the same side of the circle as heading tangent
        Point2D center_to_heading = tangent_heading - circle_center;
        Point2D center_to_target1 = tangent_target_1 - circle_center;
        Point2D center_to_target2 = tangent_target_2 - circle_center;
        
        // Use cross product to determine if points are on same side of circle
        double cross1 = center_to_heading.x * center_to_target1.y - center_to_heading.y * center_to_target1.x;
        double cross2 = center_to_heading.x * center_to_target2.y - center_to_heading.y * center_to_target2.x;
        
        // Pick the one on the same side (same sign of cross product)
        tangent_target = (cross1 * cross2 > 0) ? 
                        ((std::abs(cross1) < std::abs(cross2)) ? tangent_target_1 : tangent_target_2) :
                        tangent_target_1;
    } else {
        tangent_target = tangent_target_1;
    }
    
    // -------- Overshoot logic --------
    Point2D offset_vector = m_state.position() - tangent_heading;
    
    double dot_product = heading_dir.dot(offset_vector);

    
    if (dot_product < 0) {
        // Overshoot case
        if (m_debug) {
            std::cout << "Overshoot case detected - robot already past entry point" << std::endl;
        }
        
        m_overshoot_case = true;
        
        // Shift the center by the distance to entry point
        circle_center = circle_center + offset_vector;
        double offset = offset_vector.norm();
        
        // Derive the center of the second circle geometrically
        double x_c, y_c;
        if (use_vertical_target) {
            // Vertical target: second circle center is left of target x
            x_c = m_target.x - circle_radius;
            double dx_sq = (x_c - circle_center.x) * (x_c - circle_center.x);
            double dy_sq = std::max(0.0, (2 * circle_radius) * (2 * circle_radius) - dx_sq);
            double dy = std::sqrt(dy_sq);
            y_c = circle_center.y - dy;
        } else {
            // Horizontal target: second circle center is below target y
            y_c = m_target.y - circle_radius;
            double dy_sq = (y_c - circle_center.y) * (y_c - circle_center.y);
            double dx_sq = std::max(0.0, (2 * circle_radius) * (2 * circle_radius) - dy_sq);
            double dx = std::sqrt(dx_sq);
            x_c = circle_center.x - dx;
        }
        
        // Second circle with same radius as the first one
        Point2D circle2_center(x_c, y_c);
        
        // Used as a turning point
        Point2D circles_intersection = (circle2_center + circle_center) / 2.0;
        
        // Calculate exit point
        Point2D exit_point_1, exit_point_2;
        int num_exit = getIntersectionLineCircle(m_target_line, b_target_line, circle2_center, circle_radius,
                                                exit_point_1, exit_point_2);
        
        // Choose the exit point closer to the intersection
        Point2D exit_point = (num_exit == 2 && (exit_point_2 - circles_intersection).norm() < (exit_point_1 - circles_intersection).norm()) 
                            ? exit_point_2 : exit_point_1;
        
        // Store internal waypoints
        m_waypoints.initial = m_state.position();
        m_waypoints.turning = circles_intersection;
        m_waypoints.exit_point = exit_point;
        m_waypoints.target = m_target;
        
        // Store circle centers
        m_centers.clear();
        m_centers.push_back(circle_center);
        m_centers.push_back(circle2_center);
        
    } else {
        // Normal case - no overshoot
        m_overshoot_case = false;
        
        Point2D entry_point = tangent_heading;
        Point2D exit_point = tangent_target;
        
        // Store internal waypoints
        m_waypoints.initial = m_state.position();
        m_waypoints.turning = entry_point;
        m_waypoints.exit_point = exit_point;
        m_waypoints.target = m_target;
        
        // Store circle center
        m_centers.clear();
        m_centers.push_back(circle_center);
    }
    
    m_geometry_calculated = true;
    
    return true;
}

Point2D GeometrySolver::getPathPoint(double s)
{
    double distance_to_turn = 0.0;
    double phi_start, phi_end, phi_start2, phi_end2;
    double arc_length, arc_length2;
    
    if (m_overshoot_case) {
        distance_to_turn = 0.0;
        
        // First arc angles
        phi_start = std::atan2(m_waypoints.initial.y - m_centers[0].y, m_waypoints.initial.x - m_centers[0].x);
        phi_end = std::atan2(m_waypoints.turning.y - m_centers[0].y, m_waypoints.turning.x - m_centers[0].x);
        
        // Second arc angles
        phi_start2 = std::atan2(m_waypoints.turning.y - m_centers[1].y, m_waypoints.turning.x - m_centers[1].x);
        phi_end2 = std::atan2(m_waypoints.exit_point.y - m_centers[1].y, m_waypoints.exit_point.x - m_centers[1].x);
        
        arc_length2 = m_turning_radius * std::abs(angleDiff(phi_start2, phi_end2));
    } else {
        distance_to_turn = (m_waypoints.initial - m_waypoints.turning).norm();
        
        phi_start = std::atan2(m_waypoints.turning.y - m_centers[0].y, m_waypoints.turning.x - m_centers[0].x);
        phi_end = std::atan2(m_waypoints.exit_point.y - m_centers[0].y, m_waypoints.exit_point.x - m_centers[0].x);
        
        // There is no second arc
        arc_length2 = 0.0;
    }
    
    // First arc length
    arc_length = m_turning_radius * std::abs(angleDiff(phi_start, phi_end));
    
    // Final straight line
    double distance_to_target = (m_waypoints.target - m_waypoints.exit_point).norm();
    
    // Calculate total path length
    double path_length = distance_to_turn + arc_length + arc_length2 + distance_to_target;
    
    // Clamp s to valid range
    s = std::max(0.0, std::min(s, path_length));
    
    double x, y;
    
    // ---- Segment 1: Initial straight line ----
    if (s < distance_to_turn) {
        Point2D v = (m_waypoints.turning - m_waypoints.initial) / distance_to_turn;
        x = m_waypoints.initial.x + v.x * s;
        y = m_waypoints.initial.y + v.y * s;
    }
    // ---- Segment 2: First arc ----
    else if (s < (distance_to_turn + arc_length)) {
        double s_arc = s - distance_to_turn;
        
        double angle_sign = (angleDiff(phi_start, phi_end) >= 0) ? 1.0 : -1.0;
        double current_phi = phi_start + (s_arc / m_turning_radius) * angle_sign;
        
        Point2D c1 = m_centers[0];
        x = c1.x + m_turning_radius * std::cos(current_phi);
        y = c1.y + m_turning_radius * std::sin(current_phi);
    }
    // ---- Segment 3: Second arc (only in overshoot case) ----
    else if (arc_length2 > 0 && s < (distance_to_turn + arc_length + arc_length2)) {
        double s_arc2 = s - distance_to_turn - arc_length;
        
        double angle_sign = (angleDiff(phi_start2, phi_end2) >= 0) ? 1.0 : -1.0;
        double current_phi = phi_start2 + (s_arc2 / m_turning_radius) * angle_sign;
        
        Point2D c2 = m_centers[1];
        x = c2.x + m_turning_radius * std::cos(current_phi);
        y = c2.y + m_turning_radius * std::sin(current_phi);
    }
    // ---- Segment 4: Final straight line ----
    else if (s < path_length) {
        double s_final = s - distance_to_turn - arc_length - arc_length2;
        
        Point2D v = (m_waypoints.target - m_waypoints.exit_point) / distance_to_target;
        x = m_waypoints.exit_point.x + v.x * s_final;
        y = m_waypoints.exit_point.y + v.y * s_final;
    }
    else {
        x = m_waypoints.target.x;
        y = m_waypoints.target.y;
    }
    
    return Point2D(x, y);
}

std::vector<Point2D> GeometrySolver::generatePath(double spacing)
{
    std::vector<Point2D> path;
    
    // Calculate geometry if not already done
    if (!m_geometry_calculated) {
        if (!calculatePathGeometry()) {
            if (m_debug) {
                std::cout << "Failed to calculate path geometry" << std::endl;
            }
            return path; // Empty path
        }
    }
    
    // Calculate total path length
    double total_length = calculatePathLength();
    
    // Generate waypoints
    for (double s = 0.0; s <= total_length; s += spacing) {
        Point2D point = getPathPoint(s);
        path.push_back(point);
    }
    
    // Ensure target is included
    if (path.empty() || (path.back() - m_target).norm() > EPSILON) {
        path.push_back(m_target);
    }
    
    if (m_debug) {
        std::cout << "Generated " << path.size() << " waypoints" << std::endl;
    }
    
    return path;
}

// ========== Utility Functions Implementation ==========

double GeometrySolver::angleDiff(double a, double b) const
{
    double d = b - a;
    // Normalize to [-pi, pi]
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return d;
}

bool GeometrySolver::getTangentCircle(double m1, double b1, double m2, double b2,
                                     const Point2D& A, const Point2D& B,
                                     Point2D& vertex, Point2D& circle_center, 
                                     double& circle_radius, double radius)
{

    std::cout << "Calculating tangent circle for lines: " << std::endl;
    std::cout << "  Line 1: y = " << m1 << "*x + " << b1 << std::endl;
    std::cout << "  Line 2: y = " << m2 << "*x + " << b2 << std::endl;
    // Get intersection of the two lines
    vertex = getIntersectionLines(m1, b1, m2, b2);
    
    if (std::isinf(vertex.x) || std::isinf(vertex.y)) {
        return false;
    }
    std::cout << "Vertex (intersection): (" << vertex.x << ", " << vertex.y << ")" << std::endl;
    // Direction vectors
    Point2D vec_1 = A - vertex;
    Point2D vec_2 = B - vertex;
    
    Point2D vec_1_unit = vec_1.normalize();
    Point2D vec_2_unit = vec_2.normalize();
    
    // Bisector
    Point2D bisector = vec_1_unit + vec_2_unit;
    std::cout << "Bisector before normalization: (" << bisector.x << ", " << bisector.y << ")" << std::endl;
    Point2D bisector_unit = bisector.normalize();
    
    // Calculate the angle between bisector and line
    double dot_product = std::max(-1.0, std::min(1.0, bisector_unit.dot(vec_1_unit)));
    double half_angle = std::acos(dot_product);
    
    if (std::abs(half_angle) < EPSILON) {
        return false; // Lines are parallel or coincident
    }
    
    if (radius < 0) {
        // If radius is not provided, calculate based on distance to A
        double h = (vertex - A).norm();
        circle_radius = std::tan(half_angle) * h;
    } else {
        circle_radius = radius;
    }
    
    // Calculate distance from vertex to center
    double dist_to_center = circle_radius / std::sin(half_angle);
    
    // Place the center along the bisector
    circle_center = vertex + bisector_unit * dist_to_center;
    
    return true;
}

Point2D GeometrySolver::getIntersectionLines(double m1, double b1, double m2, double b2) const
{
    // Both lines are vertical
    if (std::isinf(m1) && std::isinf(m2)) {
        return Point2D(INF, INF);
    }
    
    // Line 1 is vertical: x = b1
    if (std::isinf(m1)) {
        return Point2D(b1, m2 * b1 + b2);
    }
    
    // Line 2 is vertical: x = b2
    if (std::isinf(m2)) {
        return Point2D(b2, m1 * b2 + b1);
    }
    
    // Check if parallel
    if (std::abs(m1 - m2) < EPSILON) {
        return Point2D(INF, INF);
    }
    
    // Solve system: y = m1*x + b1 and y = m2*x + b2
    double x = (b2 - b1) / (m1 - m2);
    double y = m1 * x + b1;
    
    return Point2D(x, y);
}

int GeometrySolver::getIntersectionLineCircle(double m, double b, const Point2D& center, double radius, Point2D& p1, Point2D& p2) const
{
    if (std::isinf(m)) {
        // Vertical line: x = b
        double x = b;
        double dx = std::abs(x - center.x);
        
        // Handle floating point precision
        if (std::abs(dx - radius) < EPSILON) {
            dx = radius;
        }
        
        // No intersection
        if (dx > radius) {
            p1 = Point2D(INF, INF);
            p2 = Point2D(INF, INF);
            return 0;
        }
        
        // One intersection (tangent)
        if (std::abs(dx - radius) < EPSILON) {
            p1 = Point2D(b, center.y);
            p2 = Point2D(INF, INF);
            return 1;
        }
        
        // Two intersections
        double dy = std::sqrt(radius * radius - dx * dx);
        p1 = Point2D(b, center.y + dy);
        p2 = Point2D(b, center.y - dy);
        return 2;
    }
    
    // Normal or horizontal line
    double A = 1 + m * m;
    double B = 2 * (m * b - m * center.y - center.x);
    double C = center.x * center.x + center.y * center.y + b * b 
              - 2 * b * center.y - radius * radius;
    
    double discriminant = B * B - 4 * A * C;
    
    // Handle floating point precision
    if (std::abs(discriminant) < EPSILON) {
        discriminant = 0.0;
    }
    
    // No intersection
    if (discriminant < 0) {
        p1 = Point2D(INF, INF);
        p2 = Point2D(INF, INF);
        return 0;
    }
    
    double sqrt_disc = std::sqrt(discriminant);
    double x1 = (-B + sqrt_disc) / (2 * A);
    double x2 = (-B - sqrt_disc) / (2 * A);
    
    double y1 = m * x1 + b;
    double y2 = m * x2 + b;
    
    p1 = Point2D(x1, y1);
    
    // One intersection (tangent)
    if (std::abs(discriminant) < EPSILON) {
        p2 = Point2D(INF, INF);
        return 1;
    }
    
    // Two intersections
    p2 = Point2D(x2, y2);
    return 2;
}

double GeometrySolver::calculatePathLength() const
{
    double distance_to_turn = 0.0;
    double arc_length, arc_length2 = 0.0;
    
    if (m_overshoot_case) {
        // First arc
        double phi_start = std::atan2(m_waypoints.initial.y - m_centers[0].y, m_waypoints.initial.x - m_centers[0].x);
        double phi_end = std::atan2(m_waypoints.turning.y - m_centers[0].y, m_waypoints.turning.x - m_centers[0].x);
        arc_length = m_turning_radius * std::abs(angleDiff(phi_start, phi_end));
        
        // Second arc
        if (m_centers.size() > 1) {
            double phi_start2 = std::atan2(m_waypoints.turning.y - m_centers[1].y, m_waypoints.turning.x - m_centers[1].x);
            double phi_end2 = std::atan2(m_waypoints.exit_point.y - m_centers[1].y, m_waypoints.exit_point.x - m_centers[1].x);
            arc_length2 = m_turning_radius * std::abs(angleDiff(phi_start2, phi_end2));
        }
    } else {
        distance_to_turn = (m_waypoints.initial - m_waypoints.turning).norm();

        double phi_start = std::atan2(m_waypoints.turning.y - m_centers[0].y, m_waypoints.turning.x - m_centers[0].x);
        double phi_end = std::atan2(m_waypoints.exit_point.y - m_centers[0].y, m_waypoints.exit_point.x - m_centers[0].x);
        arc_length = m_turning_radius * std::abs(angleDiff(phi_start, phi_end));
    }
    
    double distance_to_target = (m_waypoints.target - m_waypoints.exit_point).norm();
    
    return distance_to_turn + arc_length + arc_length2 + distance_to_target;
}

#ifndef GEOMETRY_SOLVER_H
#define GEOMETRY_SOLVER_H

#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <algorithm>

// Simple 2D point structure
struct Point2D
{
    double x;
    double y;
    double theta;  // Optional angle for target points (default 0)
    
    Point2D() : x(0.0), y(0.0), theta(0.0) {}
    Point2D(double x_, double y_, double theta_ = 0.0) : x(x_), y(y_), theta(theta_) {}
    
    Point2D operator+(const Point2D& other) const {
        return Point2D(x + other.x, y + other.y);
    }
    
    Point2D operator-(const Point2D& other) const {
        return Point2D(x - other.x, y - other.y);
    }
    
    Point2D operator*(double scalar) const {
        return Point2D(x * scalar, y * scalar);
    }
    
    Point2D operator/(double scalar) const {
        return Point2D(x / scalar, y / scalar);
    }
    
    double norm() const {
        return std::sqrt(x * x + y * y);
    }
    
    double dot(const Point2D& other) const {
        return x * other.x + y * other.y;
    }
    
    Point2D normalize() const {
        double n = norm();
        if (n < 1e-9) return Point2D(0, 0);
        return Point2D(x / n, y / n);
    }
};

// State structure: [x, y, theta]
struct RobotState
{
    double x;
    double y;
    double theta;  // heading in radians
    
    RobotState() : x(0.0), y(0.0), theta(0.0) {}
    RobotState(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
    
    Point2D position() const {
        return Point2D(x, y);
    }
};

// Internal waypoints structure
struct Waypoints
{
    Point2D initial;
    Point2D turning;
    Point2D exit_point;
    Point2D target;
};

// Path state enumeration
enum class PathState
{
    UNINITIALIZED,      // Geometry not yet calculated
    STRAIGHT_LINE,      // Simple straight line (dx=0 or dy=0)
    ANGLED_APPROACH,    // Approach with specified target angle
    NORMAL,             // Normal reverse with single arc
    OVERSHOOT,          // Overshoot case with two arcs
    FAILED              // Calculation failed (impossible geometry)
};

class GeometrySolver
{
public:
    /**
     * @brief Constructor
     * @param state Initial robot state [x, y, theta]
     * @param target Target position [x, y]
     * @param turning_radius Minimum turning radius of the robot
     */
    GeometrySolver(const RobotState& state, const Point2D& target, double turning_radius);
    

    std::vector<Point2D> generatePath(double spacing = 0.05);
    void setDebug(bool debug) { m_debug = debug; }
    
private:

    // ==========  Variables ==========
    RobotState m_state;
    Point2D m_target;
    double m_turning_radius;
    bool m_debug;
    
    // Cached geometry results
    PathState m_path_state;
    Waypoints m_waypoints;
    std::vector<Point2D> m_centers;
    
    // ========== Constants ==========
    static constexpr double INF = std::numeric_limits<double>::infinity();
    static constexpr double EPSILON = 0.5; // Tolerance for disalignment of target and heading (in meters)
    static constexpr double ANGLE_THRESHOLD = 3.0;  // Bellow this use straight line approach instead of angled approach

    // ========== Internal Path Calculation ==========
    bool calculatePathGeometry();
    Point2D getPathPoint(double s);
    
    // ========== Utility Functions ==========

    bool isAngleNearAxis(double theta);
    double normalizeAngle(double angle) const;
    double angleDiff(double a, double b) const;
    bool getTangentCircle(double m1, double b1, double m2, double b2,
                         const Point2D& A, const Point2D& B,
                         Point2D& vertex, Point2D& circle_center, 
                         double& circle_radius, double radius = -1.0);
    Point2D getIntersectionLines(double m1, double b1, double m2, double b2) const;
    int getIntersectionLineCircle(double m, double b, 
                                  const Point2D& center, double radius,
                                  Point2D& p1, Point2D& p2) const;
    double calculatePathLength() const;
    
    
};

#endif // GEOMETRY_SOLVER_H

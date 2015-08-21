#include <Eigen/Dense>

// returns the unsigned distance to the edges denoted by the vertices in V. distance[i] is the
// distance from the point p to the line defined by V.row(i) and V.row(i+1)
Eigen::Vector4d distanceToEdges(const Eigen::Matrix<double, 4, 2> &pts, const Eigen::Vector2d &p);

// the points r1 and r2 define the line, p is the point
double distanceFromLineToPoint(const Eigen::Vector2d &r1, const Eigen::Vector2d &r2, const Eigen::Vector2d &p);
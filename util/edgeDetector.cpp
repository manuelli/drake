#include "edgeDetector.h"
#include <iostream>

// returns the unsigned distance to the edges denoted by the vertices in V. distance[i] is the
// distance from the point p to the line defined by V.row(i) and V.row(i+1)
Eigen::Vector4d distanceToEdges(const Eigen::Matrix<double, 4, 2> &pts, const Eigen::Vector2d &p){
    // std::cout << "in distanceToEdges function" << std::endl;

    Eigen::Vector4d distance;
    for(int i=0; i<4; i++){
        int j = (i+1)%4;
        std::cout << "j is " << j << " i is " << i << std::endl;
        distance(i) = distanceFromLineToPoint(pts.row(i), pts.row(j), p);
    }

    std::cout << "distance is " << std::endl << distance << std::endl;

    return distance;
}

// the points r1 and r2 define the line, p is the point
double distanceFromLineToPoint(const Eigen::Vector2d &r1, const Eigen::Vector2d &r2, const Eigen::Vector2d &p){
    
    std::cout << "r1 is " << std::endl << r1 << std::endl;
    std::cout << "r2 is " << std::endl << r2 << std::endl;
    std::cout << "p is " << std::endl << p << std::endl;


    Eigen::Vector2d v = r2 - r1;
    Eigen::Vector2d w = p - r1;
    double c1 = v.dot(w);
    double c2 = v.dot(v);
    double b = c1/c2;
    Eigen::Vector2d pb = r1 + b*v;
    Eigen::Vector2d d = p - pb;

    std::cout << "distance is " << d.norm() << std::endl;
    return d.norm();
}
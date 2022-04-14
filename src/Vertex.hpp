#ifndef VERTEX_HPP_
#define VERTEX_HPP_

#include <Eigen/Core>

class Vertex {
public:
    Vertex(int id_, int x_, int y_, int theta_)
        : id(id_), x(x_), y(y_), theta(theta_) {}
    Vertex(){}
public:
    int id;
    double x;
    double y;
    double theta;
    
};

#endif
#ifndef __EDGE_HPP_
#define __EDGE_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <cmath>
#include <vector>
#include <iostream>

#include "Vertex.hpp"
#include "convert.hpp"

class Edge {
public:
    Edge() {}
    Edge(int id_from_, int id_to_, double mean_x_, double mean_y_, double mean_theta_)
        : id_from(id_from_), id_to(id_to_), mean_x(mean_x_), mean_y(mean_y_), mean_theta(mean_theta_) 
    {}

    // return A and B in a vector
    void compute_Jacobian();
    void compute_Hb();

public:
    int id_from;
    int id_to;
    double mean_x;
    double mean_y;
    double mean_theta;
    Eigen::Matrix3d inf;

    

    std::shared_ptr<Vertex> v_from;
    std::shared_ptr<Vertex> v_to;

    // H = {Hii, Hij, Hji, Hjj}
    std::vector<Eigen::Matrix3d> H;
    // b = {bi, bj}
    std::vector<Eigen::Vector3d> b;

    // A is Jacobian for i pose, B is for j pose
    std::vector<Eigen::Matrix3d> Jacobian;
    

    Eigen::Vector3d error_vector;

};


#endif
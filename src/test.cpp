#include <cmath>
#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>

#include "load_data.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void draw_odometry(std::vector<double>& x, std::vector<double>& y, std::string& title, std::string& file_name) {
    plt::figure_size(1200, 1200);
    plt::plot(x, y);
    plt::title(title);
    plt::save(file_name);
}

int main()
{   
    
    std::string v_path = "/home/yasaburo3/project/graph_slam_cpp/data/killian-v.dat";
    std::vector<Vertex> vertices;
    load_vertex_data(v_path, vertices);

    std::string e_path = "/home/yasaburo3/project/graph_slam_cpp/data/killian-e.dat";
    std::vector<Edge> edges;
    load_edge_data(e_path, edges);

    // draw the raw odometry
    int n = vertices.size();
    std::vector<double> x(n), y(n);
    for(int i = 0; i < n; i++){
        x[i] = vertices[i].x;
        y[i] = vertices[i].y;
    }
    std::string title = "raw odometry";
    std::string file_name = "../result/raw.png";
    draw_odometry(x, y, title, file_name);

    int n_vertex = vertices.size();
    int n_edge = edges.size();

    int iteration = 5;
    for(int i = 1; i < iteration;  i++){
        std:: cout << "Iteration " << i << ": " << std::endl;
    //开始迭代
        Eigen::MatrixXd H(n_vertex*3, n_vertex*3);
        Eigen::VectorXd b(n_vertex*3);
        Eigen::VectorXd delta_x(n_vertex*3);

        H = Eigen::MatrixXd::Zero(n_vertex*3, n_vertex*3);
        b = Eigen::VectorXd::Zero(n_vertex*3);
        delta_x = Eigen::VectorXd::Zero(n_vertex*3);
        // H * delta_x = b

        // Linearize
        // distribute vertex for every edge
        for(auto& edge : edges){
            int id_from = edge.id_from;
            int id_to = edge.id_to;
            edge.v_from = std::make_shared<Vertex>(vertices[id_from]);
            edge.v_to = std::make_shared<Vertex>(vertices[id_to]);

            edge.compute_Jacobian();
            edge.compute_Hb();

            std::vector<Eigen::Matrix3d> v_H = edge.H;
            std::vector<Eigen::Vector3d> v_b = edge.b;

            H.block<3,3>(id_from*3, id_from*3) += v_H[0];
            H.block<3,3>(id_from*3, id_to*3) += v_H[1];
            // H.block<3,3>(id_to*3, id_from*3) += v_H[2];
            H.block<3,3>(id_to*3, id_from*3) += v_H[1].transpose();
            H.block<3,3>(id_to*3, id_to*3) += v_H[3];
            for(int i = 0; i < 3; i++){
                b(id_from*3+i,0) += v_b[0](i,0);
            }
            for(int i = 0; i < 3; i++){
                b(id_to*3+i,0) += v_b[1](i,0);
            }

        }
        

        //Solves
        std::cout << "Pose: " << n_vertex << ", Edge: " << n_edge << std::endl;
        // keep the first node fixed
        H.block<3,3>(0,0) += Eigen::Matrix3d::Identity();

        std::vector< Eigen::Triplet<double> > triplets;
        for(int i = 0; i < H.rows(); i++){
            for(int j = 0; j < H.cols(); j++){
                if(H(i, j) * H(i, j) > 1e-5){
                    // triplets.emplace_back(i, j, H(i, j));
                    triplets.push_back(Eigen::Triplet<double>(i, j, H(i, j)));
                }
            }
        }
        Eigen::SparseMatrix<double> H_sparse(H.rows(), H.cols());
        H_sparse.setFromTriplets(triplets.begin(), triplets.end());

        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        solver.compute(H_sparse);
        if(solver.info() != Eigen::Success){
            std::cout << "ERROR during decomposition!!" << std::endl;
        }
        delta_x = solver.solve(b);
        if(solver.info() != Eigen::Success){
            std::cout << "ERROR during solving!!" << std::endl;
        }

        for(int t = 0; t < n_vertex; t++){
            vertices[t].x += delta_x(t*3,0);
            vertices[t].y += delta_x(t*3+1,0);
            vertices[t].theta += delta_x(t*3+2,0);
        }

    //迭代结束

    // draw new odometry
    int n = vertices.size();
    std::vector<double> x(n), y(n);
    for(int i = 0; i < n; i++){
        x[i] = vertices[i].x;
        y[i] = vertices[i].y;
    }
    std::string title = "iteration " + std::to_string(i);
    std::string file_name = "../result/" + std::to_string(i) + ".png";
    draw_odometry(x, y, title, file_name);
    
    }

    return 0;
}
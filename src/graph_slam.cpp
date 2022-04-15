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

// g2o
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_impl.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

namespace plt = matplotlibcpp;

void draw_odometry(std::vector<double>& x, std::vector<double>& y, std::string& title, std::string& file_name) {
    plt::figure_size(1200, 1200);
    plt::plot(x, y);
    plt::title(title);
    plt::save(file_name);
}

void optimizeWithEigen(std::vector<Vertex>& vertices, std::vector<Edge>& edges, int iteration) {
    int n_vertex = vertices.size();
    int n_edge = edges.size();

    // Eigen库实现的图优化
    // int iteration = 5;
    // for(int i = 1; i < iteration;  i++){
        // std:: cout << "Iteration " << iteration << ": " << std::endl;
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
    std::string title = "iteration " + std::to_string(iteration);
    std::string file_name = "../result/Eigen/" + std::to_string(iteration) + ".png";
    draw_odometry(x, y, title, file_name);
    
    // }
}

void optimizeWithG2O(std::vector<Vertex>& vertices, std::vector<Edge>& edges) {
    // g2o settings
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,3>> Block;
    auto linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<Block::PoseMatrixType>>();
    auto solver_ptr = g2o::make_unique<Block>(std::move(linearSolver));

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // add vertices to graph of g2o
    std::vector<g2o::VertexSE2*> vecVertex;
    for(int i = 0; i < vertices.size(); i++){
        g2o::VertexSE2* v = new g2o::VertexSE2();
        v->setEstimate(g2o::SE2(vertices[i].x, vertices[i].y, vertices[i].theta));
        v->setId(i);
        if(i == 0){
            v->setFixed(true);
        }
        optimizer.addVertex(v);
        vecVertex.push_back(v);
    }
    
    // add edged to graph of g2o
    for(int i = 0; i < edges.size(); i++){
        g2o::EdgeSE2* e = new g2o::EdgeSE2();
        e->setId(i);
        e->setVertex(0, vecVertex[edges[i].id_from]);
        e->setVertex(1, vecVertex[edges[i].id_to]);
        e->setMeasurement(g2o::SE2(edges[i].mean_x, edges[i].mean_y, edges[i].mean_theta));
        e->setInformation(edges[i].inf);
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
        e->setRobustKernel(rk);
        optimizer.addEdge(e);
    }

    std::cout << "g2o optimizer has been built!" << std::endl;
    optimizer.save("../result/g2o/g2o_before.g2o");
    optimizer.initializeOptimization();
    optimizer.optimize(20);        // 后面可以调小一点
    optimizer.save("../result/g2o/g2o_after.g2o");
}

int main()
{   
    
    std::string v_path = "../data/killian-v.dat";
    std::vector<Vertex> vertices;
    load_vertex_data(v_path, vertices);

    std::string e_path = "../data/killian-e.dat";
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
    std::string file_name = "../result/Eigen/raw.png";
    draw_odometry(x, y, title, file_name);

    int iteration_num = 5;
    for(int i = 1; i < iteration_num; i++){
        std:: cout << "Iteration " << i << ": " << std::endl;
        optimizeWithEigen(vertices, edges, i);
    }

    optimizeWithG2O(vertices, edges);


    return 0;
}
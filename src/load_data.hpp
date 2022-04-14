#ifndef LOAD_DATA_HPP_
#define LOAD_DATA_HPP
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Core>

#include "Edge.hpp"
#include "Vertex.hpp"
bool load_vertex_data(const std::string& path, std::vector<Vertex>& vertices){
    std::ifstream fin(path);
    if(!fin.is_open()){
        std::cout << "Invalid file path to vertex data: " << path << std::endl;
        return false;
    }

    while(!fin.eof()){
        std::string str;
        std::getline(fin, str);
        std::stringstream ss(str);
        std::vector<std::string> words(5);
        for(int i = 0; i < words.size(); i++){
            ss >> words[i];
        }
        Vertex v;
        v.id = atoi(words[1].c_str());
        v.x = atof(words[2].c_str());
        v.y = atof(words[3].c_str());
        v.theta = atof(words[4].c_str());

        if(v.id == 0 && v.x == 0 && v.y == 0 && v.theta == 0)
            continue;
        vertices.push_back(v);
    }

    return true;
}

bool load_edge_data(const std::string& path, std::vector<Edge>& edges) {
    std::ifstream fin(path);
    if(!fin.is_open()){
        std::cout << "Invalid file path to edge data: " << path << std::endl;
        return false;
    }

    while(!fin.eof()){
        std::string str;
        std::getline(fin, str);
        std::stringstream ss(str);
        std::vector<std::string> words(12);
        for(int i = 0; i < words.size(); i++){
            ss >> words[i];
        }
        std::vector<double> nums;
        for(int i = 1; i < words.size(); i++){
            double temp = atof(words[i].c_str());
            nums.push_back(temp);
        }

        Edge e;
        e.id_from = static_cast<int>(nums[0]);
        e.id_to = static_cast<int>(nums[1]);
        e.mean_x = nums[2];
        e.mean_y = nums[3];
        e.mean_theta = nums[4];
        Eigen::Matrix3d inf;
        inf(0,0) = nums[5];
        inf(1,0) = nums[6];
        inf(0,1) = nums[6];
        inf(1,1) = nums[7];
        inf(2,2) = nums[8];
        inf(0,2) = nums[9];
        inf(2,0) = nums[9];
        inf(2,1) = nums[10];
        inf(1,2) = nums[10];
        e.inf = inf;

        if(e.id_from == 0 && e.id_to == 0 && e.mean_x == 0 && e.mean_y == 0 
            && e.mean_theta == 0)
            continue;

        edges.push_back(e);
    }
}

#endif
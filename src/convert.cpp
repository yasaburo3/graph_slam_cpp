#include "convert.hpp"

// 位姿变换矩阵->向量
Eigen::Vector3d t2v(const Eigen::Matrix3d& A) {
    // A = cos -sin x
    //     sin cos  y
    //      0   0   1
    Eigen::Vector3d v;
    v(0, 0) = A(0, 2);
    v(1, 0) = A(1, 2);
    v(2, 0) = atan2(A(1, 0), A(1, 1));
    return v;

}

// 向量->位姿变换矩阵
Eigen::Matrix3d v2t(const Eigen::Vector3d& v){
    // v = x y theta
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    double c = cos(v(2, 0));
    double s = sin(v(2, 0));
    A(0,0) = c;
    A(0,1) = -s;
    A(0,2) = v(0,0);
    A(1,0) = s;
    A(1,1) = c;
    A(1,2) = v(1,0);
    A(2,0) = 0;
    A(2,1) = 0;
    A(2,2) = 1;
    return A;
}

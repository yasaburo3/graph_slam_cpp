#include "Edge.hpp"

// A is Jacobian for i pose, B is for j pose
void Edge::compute_Jacobian() {
    Eigen::Vector3d z_ij(mean_x, mean_y, mean_theta);
    Eigen::Vector3d v_i(v_from->x, v_from->y, v_from->theta);
    Eigen::Vector3d v_j(v_to->x, v_to->y, v_to->theta);

    Eigen::Matrix3d zt_ij = v2t(z_ij);
    Eigen::Matrix3d vt_i = v2t(v_i);
    Eigen::Matrix3d vt_j = v2t(v_j);

    Eigen::Matrix3d f_ij = vt_i.inverse() * vt_j;

    double theta_i = v_from->theta;
    double dx_ij = v_j(0,0) - v_i(0,0);
    double dy_ij = v_j(1,0) - v_i(1,0);

    double si = sin(theta_i);
    double ci = cos(theta_i);

    Eigen::Matrix3d A;
    A(0,0) = -ci;
    A(0,1) = -si;
    A(1,0) = si;
    A(1,1) = -ci;
    A(0,2) = -si * dx_ij + ci * dy_ij;
    A(1,2) = -ci * dx_ij - si * dy_ij;
    A(2,0) = 0;
    A(2,1) = 0;
    A(2,2) = -1;

    Eigen::Matrix3d B;
    B << ci, si, 0, -si, ci, 0, 0, 0, 1;

    // multiply inverse of Rz to A and B
    Eigen::Matrix3d zt_inv = zt_ij.inverse();
    // error vector
    error_vector = t2v(zt_inv * f_ij);
    // make the translation of z zero, 因为对x和y的位姿求偏导
    zt_inv(0,2) = 0;
    zt_inv(1,2) = 0;
    A = zt_inv * A;
    B = zt_inv * B;

    Jacobian = {A, B};
}

void Edge::compute_Hb() {
    Eigen::Matrix3d A = Jacobian[0];
    Eigen::Matrix3d B = Jacobian[1];

    Eigen::Matrix3d Hii, Hij, Hji, Hjj;
    Eigen::Vector3d bi, bj;
    bi = -A.transpose() * inf * error_vector;
    bj = -B.transpose() * inf * error_vector;
    Hii = A.transpose() * inf * A;
    Hij = A.transpose() * inf * B;
    Hji = B.transpose() * inf * A;
    Hjj = B.transpose() * inf * B;

    H.clear();
    H.push_back(Hii);
    H.push_back(Hij);
    H.push_back(Hji);
    H.push_back(Hjj);

    b.clear();
    b.push_back(bi);
    b.push_back(bj);

}


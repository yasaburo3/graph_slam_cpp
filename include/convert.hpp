#ifndef __CONVERT_HPP_
#define __CONVERT_HPP_

#include <Eigen/Core>
#include <cmath>

Eigen::Vector3d t2v(const Eigen::Matrix3d& A);
Eigen::Matrix3d v2t(const Eigen::Vector3d& v);


#endif
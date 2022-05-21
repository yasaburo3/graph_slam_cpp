#ifndef __ERROR_FUNCTION_HPP_
#define __ERROR_FUNCTION_HPP_

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cmath>
#include "normalize_angle.hpp"

template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians) {
    const T cos_yaw = ceres::cos(yaw_radians);
    const T sin_yaw = ceres::sin(yaw_radians);

    Eigen::Matrix<T, 2, 2> rotation;
    rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
    return rotation;
}

class error_function {
public:
    error_function(const double x_ab, const double y_ab, const double theta_ab, 
                   const Eigen::Matrix3d& sqrt_information)
            : p_ab_(x_ab, y_ab), theta_ab_(theta_ab),
              sqrt_information_(sqrt_information) {}

    template<typename T>
    bool operator()(const T* const x_a, const T* const y_a, const T* const theta_a,
                    const T* const x_b, const T* const y_b, const T* const theta_b,
                    T* residuals_ptr) const {
        const Eigen::Matrix<T, 2, 1> p_a(*x_a, *y_a);
        const Eigen::Matrix<T, 2, 1> p_b(*x_b, *y_b);

        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals_ptr);

        residuals_map.template head<2>() = 
            RotationMatrix2D(*theta_a).transpose() * (p_b - p_a) - p_ab_.cast<T>();
        residuals_map(2) = NormalizeAngle(
            (*theta_b - *theta_a) - static_cast<T>(theta_ab_));

        residuals_map = sqrt_information_.template cast<T>() * residuals_map;

        return true;
    }

private:
    const Eigen::Vector2d p_ab_;
    const double theta_ab_;
    const Eigen::Matrix3d sqrt_information_;
};


#endif
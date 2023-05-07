#ifndef _COMMON_TYPE_NAME_
#define _COMMON_TYPE_NAME_

#include "Eigen/Dense"
#include "fmt/color.h"
#include "utils/scope_timer.hpp"
#include "ros/ros.h"

using namespace fmt;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 3, 3> Mat33;

typedef Eigen::Matrix<double, 3, 3> StatePVA;
typedef Eigen::Matrix<double, 3, 4> StatePVAJ;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DynamicMat;
typedef Eigen::MatrixX4d MatX4;
typedef std::pair<double, Vec3> SamplePoint;

typedef Eigen::Matrix3Xd PolyhedronV;
typedef Eigen::MatrixX4d PolyhedronH;

struct QuadState {
    Vec3 position, velocity, acceleration, jerk;
    double yaw;
    ros::Time callback_time;
    bool rcv{false};
    Eigen::Quaterniond q;
} ;

#endif
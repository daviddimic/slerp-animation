#ifndef __ISOMETRY__
#define __ISOMETRY__ 1

#include <vector>
#include <Eigen/Geometry>


Eigen::Matrix3d Euler2A(double phi, double theta, double psi);

std::pair<Eigen::Vector3d, double> A2AxisAngle(const Eigen::Matrix3d& A);

Eigen::Matrix3d Rodrigues(const Eigen::Vector3d& p, double phi);

Eigen::Vector3d A2Euler(const Eigen::Matrix3d& A);

Eigen::Quaterniond AxisAngle2Q(const Eigen::Vector3d& p, double phi);

std::pair<Eigen::Vector3d, double> Q2AxisAngle(const Eigen::Quaterniond& q);

Eigen::Quaterniond slerp(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double tm, double t);


#endif /* ifndef __ISOMETRY__ */

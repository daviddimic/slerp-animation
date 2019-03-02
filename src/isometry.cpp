#include <iostream>
#include <cstdlib>

#include "isometry.hpp"

#define EPS 1e-5


static bool isIsometry(const Eigen::Matrix3d& A) {
	if (!(A * A.transpose()).isApprox(Eigen::Matrix3d::Identity(), EPS)) {
		std::cerr << "Error! A is not orthogonal" << std::endl;
		return false;
	}

	double det = A.determinant();
	if (!(det >= 1 - EPS && det <= 1 + EPS)) {
		std::cerr << "Error! A determinant is not 1" << std::endl;
		return false;
	}

	return true;
}


Eigen::Matrix3d Euler2A(double phi, double theta, double psi) {
	if (!(std::abs(phi) >= 0 && std::abs(phi) < 2*M_PI)) {
		std::cerr << "Error! phi shoud be from [0, 2*PI)" << std::endl;
		exit(EXIT_FAILURE);
	}
	if (!(std::abs(psi) >= 0 && std::abs(psi) < 2*M_PI)) {
		std::cerr << "Error! psi shoud be from [0, 2*PI)" << std::endl;
		exit(EXIT_FAILURE);
	}
	if (!(theta >= -M_PI/2.0 && theta <= M_PI/2.0)) {
		std::cerr << "Error! theta shoud be from [-PI/2, PI/2]" << std::endl;
		exit(EXIT_FAILURE);
	}


	Eigen::Matrix3d Rx;
	Rx << 	1.0, 0.0, 0.0,
			0.0, std::cos(phi), -std::sin(phi),
			0.0, std::sin(phi), std::cos(phi);

	Eigen::Matrix3d Ry;
	Ry << 	std::cos(theta), 0.0, std::sin(theta),
			0.0, 1.0, 0.0,
			-std::sin(theta), 0.0, std::cos(theta);

	Eigen::Matrix3d Rz;
	Rz << 	std::cos(psi), -std::sin(psi), 0.0,
			std::sin(psi), std::cos(psi), 0.0,
			0.0, 0.0, 1.0;

	return Rz * Ry * Rx;
}


std::pair<Eigen::Vector3d, double> A2AxisAngle(const Eigen::Matrix3d& A) {
	if (!isIsometry(A)) {
		exit(EXIT_FAILURE);
	}

	Eigen::Matrix3d Ap = A - Eigen::Matrix3d::Identity();
	Eigen::Vector3d p1 = Ap.row(0);
	Eigen::Vector3d p2 = Ap.row(1);
	Eigen::Vector3d u = Ap.row(2).normalized();
	Eigen::Vector3d p = p1.cross(p2);
	p.normalize();

	Eigen::Vector3d up = A * u;

	double phi = std::acos(u.dot(up));

	Eigen::Matrix3d mixedProduct;
	mixedProduct.row(0) = u;
	mixedProduct.row(1) = up;
	mixedProduct.row(2) = p;
	if (mixedProduct.determinant() < 0) {
		p = -p;
	}

	return std::pair<Eigen::Vector3d, double>(p, phi);
}


Eigen::Matrix3d Rodrigues(const Eigen::Vector3d& p, double phi) {
	if (!p.isUnitary(EPS)) {
		std::cerr << "Error! p norm is not 1" << std::endl;
		exit(EXIT_FAILURE);
	}
	if (!(std::abs(phi) >= 0 && std::abs(phi) <= M_PI)) {
		std::cerr << "Error! phi shoud be from [0, PI]" << std::endl;
		exit(EXIT_FAILURE);
	}

	auto ppt = p * p.transpose();
	Eigen::Matrix3d px;
	px <<	0.0, -p(2), p(1),
			p(2), 0.0, -p(0),
			-p(1), p(0), 0.0;
	Eigen::Matrix3d Rp = ppt + std::cos(phi) * (Eigen::Matrix3d::Identity() - ppt) + std::sin(phi) * px;
	return Rp;
}


Eigen::Vector3d A2Euler(const Eigen::Matrix3d& A) {
	if (!isIsometry(A)) {
		exit(EXIT_FAILURE);
	}

	double psi, theta, phi;

	if (A(2, 0) < 1) {
		if (A(2, 0) > -1) {
			psi = std::atan2(A(1, 0), A(0, 0));
			theta = std::asin(-A(2, 0));
			phi = std::atan2(A(2, 1), A(2, 2));
		} else {
			// -1
			psi = atan2(-A(0, 1), A(1, 1));
			theta = M_PI / 2.0;
			phi = 0;
		}
	} else {
		// 1
		psi = std::atan2(-A(0, 1), A(1, 1));
		theta = -M_PI / 2.0;
		phi = 0;
	}

	return Eigen::Vector3d(phi, theta, psi);
}


Eigen::Quaterniond AxisAngle2Q(const Eigen::Vector3d& p, double phi) {
	if (!p.isUnitary(EPS)) {
		std::cerr << "Error! p norm is not 1" << std::endl;
		exit(EXIT_FAILURE);
	}
	if(!(std::abs(phi) >= 0 && std::abs(phi) < 2*M_PI)){
		std::cerr << "Error! phi shoud be from [0, 2PI)" << std::endl;
		exit(EXIT_FAILURE);
	}

	double w = std::cos(phi/2.0);
	Eigen::Vector3d p1 = p.normalized();
	Eigen::Vector3d v = std::sin(phi/2.0) * p1;

	return Eigen::Quaterniond(w, v[0], v[1], v[2]);
}


std::pair<Eigen::Vector3d, double> Q2AxisAngle(const Eigen::Quaterniond& q) {
	auto q2 = q.normalized();

	double w = q2.w();

	if (w < 0) {
		q2.w() = -w;
	}

	double phi = 2*std::acos(w);

	Eigen::Vector3d p;
	if (std::abs(w) == 1) {
		p = {1, 0, 0};
	} else {
		Eigen::Vector3d im(q2.x(), q2.y(), q2.z());
		p = im.normalized();
	}

	return std::pair<Eigen::Vector3d, double>(p, phi);
}


Eigen::Quaterniond slerp(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double tm, double t) {

	Eigen::Quaterniond q11 = q1.normalized();
	Eigen::Quaterniond q22 = q2.normalized();

	double cos0 = q1.dot(q2);

	if (cos0 < 0) {
		q11.w() *= -1;
		q11.vec() *= -1;

		cos0 = -cos0;
	}
	if (cos0 > 0.95) {
		return q11;
	}

	double phi0 = std::acos(cos0);

	double q1Coeff = std::sin(phi0 * (1 - t / tm)) / std::sin(phi0);
	double q2Coeff = std::sin(phi0 * (t / tm)) / std::sin(phi0);

	Eigen::Quaterniond qs_t;
	qs_t.w() = q1Coeff * q11.w() + q2Coeff * q22.w();
	qs_t.vec() = q1Coeff * q11.vec() + q2Coeff * q22.vec();

	return qs_t;
}

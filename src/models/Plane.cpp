//
//  Plane.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 11.02.2017.
//
//

#include "../../include/models/Plane.h"

Plane::Plane() {
    A = 0;
    B = 0;
    C = 0;
    D = 0;
}

Plane::Plane(Eigen::Vector3f point1, Eigen::Vector3f point2, Eigen::Vector3f point3) {
    computePlaneEquation(point1, point2, point3);
}

Plane::Plane(std::array<Eigen::Vector3f, 3>) {

}

Plane::Plane(Eigen::Vector3f normalVec, Eigen::Vector3f point) {
    A = normalVec(0);
    B = normalVec(1);
    C = normalVec(2);
    D = A * point(0) + B * point(1) + C * point(2);
    valid = true;
}

float Plane::getA() {
    return A;
}

float Plane::getB() {
    return B;
}

float Plane::getC() {
    return C;
}

float Plane::getD() {
    return D;
}

float Plane::getDistanceFromPoint(Eigen::Vector3f point) {
    return std::abs(A * point(0) + B * point(1) + C * point(2) - D)
           / sqrtf(powf(A, 2) + powf(B, 2) + powf(C, 2));
}

void Plane::computePlaneEquation(Eigen::Vector3f point1, Eigen::Vector3f point2, Eigen::Vector3f point3) {
    Eigen::Vector3f v = point1 - point2;
    Eigen::Vector3f w = point1 - point3;
    Eigen::Vector3f planeParameters = v.cross(w);
    A = planeParameters(0);
    B = planeParameters(1);
    C = planeParameters(2);
    D = A * point1(0) + B * point1(1) + C * point1(2);
    valid = true;
}

float Plane::isValid() const {
    return valid;
}

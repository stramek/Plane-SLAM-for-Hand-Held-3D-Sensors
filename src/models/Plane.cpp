//
//  Plane.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 11.02.2017.
//
//

#include "../../include/models/Plane.h"

Plane::Plane() {}

Plane::Plane(Eigen::Vector3f point1, Eigen::Vector3f point2, Eigen::Vector3f point3, const Mat& colorImage) {
    color = HSVColor(colorImage);
}

Plane::Plane(std::array<Eigen::Vector3d, 3>, const Mat& colorImage) {
    color = HSVColor(colorImage);
}

Plane::Plane(Eigen::Vector3f normalVec, Eigen::Vector3f point, const Mat& colorImage) {
    A = normalVec(0);
    B = normalVec(1);
    C = normalVec(2);
    D = A * point(0) + B * point(1) + C * point(2);
    color = HSVColor(colorImage);
    valid = true;
}

double Plane::getA() {
    return A;
}

double Plane::getB() {
    return B;
}

double Plane::getC() {
    return C;
}

double Plane::getD() {
    return D;
}

double Plane::getDistanceFromPoint(Eigen::Vector3f point) {
    return std::abs(A * point(0) + B * point(1) + C * point(2) + D)
           / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));
}

void Plane::computePlaneEquation(Eigen::Vector3d point1, Eigen::Vector3d point2, Eigen::Vector3d point3) {
    Eigen::Vector3d v = point1 - point2;
    Eigen::Vector3d w = point1 - point3;
    Eigen::Vector3d planeParameters = v.cross(w);
    A = planeParameters(0);
    B = planeParameters(1);
    C = planeParameters(2);
    D = A * point1(0) + B * point1(1) + C * point1(2);
    valid = true;
}

bool Plane::isValid() const {
    return valid;
}

const HSVColor &Plane::getColor() const {
    return color;
}

Eigen::Vector3f Plane::computePointOnPlaneFromTwoCoordinate(float x, float y) const {
    Eigen::Vector3f pointToReturn;
    float z = (D - A * x - B * y) / C;
    pointToReturn(0) = x;
    pointToReturn(1) = y;
    pointToReturn(2) = z;
    return  pointToReturn;
}

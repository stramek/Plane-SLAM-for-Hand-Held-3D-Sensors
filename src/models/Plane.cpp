//
//  Plane.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 11.02.2017.
//
//

#include "../../include/models/Plane.h"

Plane::Plane() {}

Plane::Plane(Vector3f normalVec, Vector3f point, const Mat& colorImage, const vector<Vector3f> &points, const ImageCoords &imageCoords) {
    A = normalVec(0);
    B = normalVec(1);
    C = normalVec(2);
    D = A * point(0) + B * point(1) + C * point(2);
    color = HSVColor(colorImage);
    this->points = points;
    this->imageCoords = imageCoords;
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

double Plane::getDistanceFromPoint(Eigen::Vector3d point) {
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

const ImageCoords &Plane::getImageCoords() const {
    return imageCoords;
}

void Plane::setImageCoords(const ImageCoords &imageCoords) {
    Plane::imageCoords = imageCoords;
}

const vector<Vector3f> &Plane::getPoints() const {
    return points;
}

void Plane::setPoints(const vector<Vector3f> &points) {
    Plane::points = points;
}

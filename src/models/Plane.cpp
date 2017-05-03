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
    planeNormalVec = normalVec;
    planeNormalVec.normalize();
    A = normalVec(0);
    B = normalVec(1);
    C = normalVec(2);
    D = A * point(0) + B * point(1) + C * point(2);
    color = HSVColor(colorImage);
    valid = true;
}

Plane::Plane(Eigen::Vector3f normalVec, float D){
    planeNormalVec = normalVec;
    planeNormalVec.normalize();
    A = normalVec(0);
    B = normalVec(1);
    C = normalVec(2);
    this->D = D;
    valid = true;
}

float Plane::getA() const {
    return A;
}

float Plane::getB() const {
    return B;
}

float Plane::getC() const {
    return C;
}

float Plane::getD() const {
    return D;
}

float Plane::getDistanceFromPoint(Eigen::Vector3f point) {
    return std::abs(A * point(0) + B * point(1) + C * point(2) + D)
           / sqrtf(powf(A, 2) + powf(B, 2) + powf(C, 2));
}

void Plane::computePlaneEquation(Eigen::Vector3f point1, Eigen::Vector3f point2, Eigen::Vector3f point3) {
    Eigen::Vector3f v = point1 - point2;
    Eigen::Vector3f w = point1 - point3;
    Eigen::Vector3f planeParameters = v.cross(w);
    planeParameters.normalize();
    A = planeParameters(0);
    B = planeParameters(1);
    C = planeParameters(2);
    D = A * point1(0) + B * point1(1) + C * point1(2);
    valid = true;
    planeNormalVec(0) = A;
    planeNormalVec(1) = B;
    planeNormalVec(2) = C;
}

bool Plane::isValid() const {
    return valid;
}

const HSVColor &Plane::getColor() const {
    return color;
}

Eigen::Vector3f Plane::computePointOnPlaneFromTwoCoordinate(float firstCoordinate, float secondCoordinate) const {
    Eigen::Vector3f pointToReturn;
    if(A == 0 && B == 0 && C == 0){
        throw std::runtime_error("Plane equation error!");
    } else if( A == 0 && B == 0){
        float z = D / B;
        pointToReturn(0) = firstCoordinate;
        pointToReturn(1) = secondCoordinate;
        pointToReturn(2) = z;
    } else if(A == 0 && C == 0){
        float y = D / B;
        pointToReturn(0) = firstCoordinate;
        pointToReturn(1) = y;
        pointToReturn(2) = secondCoordinate;
    } else if(B==0 && C==0){
        float x = D / A;
        pointToReturn(0) = x;
        pointToReturn(1) = firstCoordinate;
    } else if(C == 0){
        float y = (D - A * firstCoordinate) / B;
        pointToReturn(0) = firstCoordinate;
        pointToReturn(1) = y;
        pointToReturn(2) = secondCoordinate;
    } else{
        float z = (D - A * firstCoordinate - B * secondCoordinate) / C;
        pointToReturn(0) = firstCoordinate;
        pointToReturn(1) = secondCoordinate;
        pointToReturn(2) = z;
    }
    return  pointToReturn;
}

Eigen::Vector3f Plane::getPlaneNormalVec() const{
    return planeNormalVec;
}

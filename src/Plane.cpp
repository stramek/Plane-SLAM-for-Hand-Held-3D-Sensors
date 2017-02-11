//
//  Plane.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 11.02.2017.
//
//

#include "../include/Plane.h"

Plane::Plane() {}

Plane::Plane(cv::Point3d point1, cv::Point3d point2, cv::Point3d point3) {
    
}

Plane::Plane(std::array<cv::Point3d, 3>) {
    
}

double Plane::getA() {
    return A;
}

double Plane::getB() {
    return B;
}

double Plane::getD() {
    return D;
}

double Plane::getDistanceFromPoint(cv::Point3d point) {
    return abs(A * point.x + B * point.y + point.z + D)
            / sqrt(pow(A, 2) + pow(B, 2) + 1);
}

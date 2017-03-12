//
//  Plane.hpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 11.02.2017.
//
//

#ifndef Plane_h
#define Plane_h
#ifdef __GNUC__
// Avoid tons of warnings with root code
#pragma GCC system_header
#endif

#include <opencv2/opencv.hpp>
#include <array>
#include <Eigen/Dense>

class Plane {
public:
    Plane();
    Plane(Eigen::Vector3d point1, Eigen::Vector3d  point2, Eigen::Vector3d  point3);
    Plane(std::array<Eigen::Vector3d , 3>);
    Plane(Eigen::Vector3f normalVec, Eigen::Vector3f point);
    double getA();
    double getB();
    double getC();
    double getD();
    bool isValid() const;
    double getDistanceFromPoint(Eigen::Vector3d  point);
private:
    double A, B, D, C;
    bool valid = false;
    void computePlaneEquation(Eigen::Vector3d  point1, Eigen::Vector3d  point2, Eigen::Vector3d  point3);
};

#endif /* Plane_h */
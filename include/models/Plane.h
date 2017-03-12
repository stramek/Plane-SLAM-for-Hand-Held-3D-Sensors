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

    Plane(Eigen::Vector3f point1, Eigen::Vector3f point2, Eigen::Vector3f point3);

    Plane(std::array<Eigen::Vector3f, 3>);

    Plane(Eigen::Vector3f normalVec, Eigen::Vector3f point);

    float getA();

    float getB();

    float getC();

    float getD();

    float isValid() const;

    float getDistanceFromPoint(Eigen::Vector3f point);

private:
    float A, B, D, C;
    bool valid = false;

    void computePlaneEquation(Eigen::Vector3f point1, Eigen::Vector3f point2, Eigen::Vector3f point3);
};

#endif /* Plane_h */

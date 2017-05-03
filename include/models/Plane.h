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

#include "include/utils/HSVColor.h"
#include <opencv2/opencv.hpp>
#include <array>
#include <Eigen/Dense>

class Plane {
public:
    Plane();
    Plane(Eigen::Vector3f point1, Eigen::Vector3f  point2, Eigen::Vector3f  point3, const Mat& colorImage);
    Plane(std::array<Eigen::Vector3d , 3>, const Mat& colorImage);
    Plane(Eigen::Vector3f normalVec, Eigen::Vector3f point, const Mat& colorImage);
    float getA();
    float getB();
    float getC();
    float getD();
    bool isValid() const;
    float getDistanceFromPoint(Eigen::Vector3f  point);
    Eigen::Vector3f getPlaneNormalVec() const;
    Eigen::Vector3f computePointOnPlaneFromTwoCoordinate(float x = 0, float y = 0) const;

    const HSVColor &getColor() const;

private:
    float A, B, D, C;
    Eigen::Vector3f planeNormalVec;
    HSVColor color = HSVColor();
    bool valid = false;
    void computePlaneEquation(Eigen::Vector3f  point1, Eigen::Vector3f  point2, Eigen::Vector3f  point3);
};

#endif /* Plane_h */

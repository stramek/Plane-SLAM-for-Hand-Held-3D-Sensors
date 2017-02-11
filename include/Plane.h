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

class Plane {
public:
    Plane();
    Plane(cv::Point3d point1, cv::Point3d point2, cv::Point3d point3);
    Plane(std::array<cv::Point3d, 3>);
    double getA();
    double getB();
    double getC();
    double getD();
    double getDistanceFromPoint(cv::Point3d point);
private:
    double A, B, D, C;
};

#endif /* Plane_h */

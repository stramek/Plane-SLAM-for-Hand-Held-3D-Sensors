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

#include "include/models/HSVColor.h"
#include "include/models/ImageCoords.h"
#include "include/models/Point3D.h"
#include <opencv2/opencv.hpp>
#include <array>
#include <Eigen/Dense>

using namespace Eigen;

class Plane {
public:
    Plane();
    Plane(Vector3f normalVec, Vector3f point, const Mat& colorImage, const vector<Vector3f> &points, const ImageCoords &imageCoords);
    double getA();
    double getB();
    double getC();
    double getD();
    bool isValid() const;
    double getDistanceFromPoint(Eigen::Vector3d  point);
    const ImageCoords &getImageCoords() const;
    void setImageCoords(const ImageCoords &imageCoords);
    const HSVColor &getColor() const;
    const vector<Vector3f> &getPoints() const;
    void setPoints(const vector<Vector3f> &points);

private:
    double A, B, D, C;
    vector<Vector3f> points;
    ImageCoords imageCoords = ImageCoords();
    HSVColor color = HSVColor();
    bool valid = false;
    void computePlaneEquation(Eigen::Vector3d  point1, Eigen::Vector3d  point2, Eigen::Vector3d  point3);
};

#endif /* Plane_h */

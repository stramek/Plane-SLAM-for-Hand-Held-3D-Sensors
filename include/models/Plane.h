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
    Plane(Vector3f point1, Vector3f  point2, Vector3f  point3, const Mat& colorImage);
    Plane(std::array<Eigen::Vector3d , 3>, const Mat& colorImage);
    Plane(Vector3f normalVec, Vector3f point, const Mat& colorImage, const vector<Vector3f> &points, const ImageCoords &imageCoords);
    Plane(Vector3f normalVec, float D); // added for testing, TODO remove after testing
    float getA() const;
    float getB() const;
    float getC() const;
    float getD() const;
    bool isValid() const;
    float getDistanceFromPoint(Eigen::Vector3f  point);
    Vector3f getPlaneNormalVec() const;
    Vector3f computePointOnPlaneFromTwoCoordinate(float firstCoordinate = 0, float secondCoordinate = 0) const;

    const ImageCoords &getImageCoords() const;
    void setImageCoords(const ImageCoords &imageCoords);
    const HSVColor &getColor() const;
    const vector<Vector3f> &getPoints() const;
    void setPoints(const vector<Vector3f> &points);
    unsigned int getNumberOfPoints() const;

private:
    float A, B, D, C;
    vector<Vector3f> points;
    ImageCoords imageCoords = ImageCoords();
    Vector3f planeNormalVec;
    HSVColor color = HSVColor();
    bool valid = false;
    void computePlaneEquation(Vector3f  point1, Vector3f  point2, Vector3f  point3);
};

#endif /* Plane_h */

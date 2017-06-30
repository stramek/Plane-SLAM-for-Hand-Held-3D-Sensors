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

    Plane(const Vector3d &point1, const Vector3d &point2, const Vector3d &point3, const ImageCoords &imageCoords);

    Plane(std::array<Eigen::Vector3d, 3>, const Mat &colorImage);

    Plane(Vector3d normalVec, Vector3d point, const vector<Point3D> &points, const ImageCoords &imageCoords);

    Plane(Vector3d normalVec, double D, vector<Point3D> points, vector<ImageCoords> imageCoordsVec, HSVColor color);

    Plane(double D, const Vector3d &planeNormalVec);

    double getA() const;

    double getB() const;

    double getC() const;

    double getD() const;

    bool isValid() const;

    double getDistanceFromPoint(Eigen::Vector3d point) const;

    Vector3d getPlaneNormalVec() const;

    Vector3d computePointOnPlaneFromTwoCoordinate(double firstCoordinate = 0, double secondCoordinate = 0) const;

    const ImageCoords &getImageCoords() const;

    void setImageCoords(const ImageCoords &imageCoords);

    const HSVColor &getColor() const;

    const vector<Point3D> &getPoints() const;

    void setPoints(const vector<Point3D> &points);

    unsigned int getNumberOfPoints() const;

    double getAngleBetweenTwoPlanes(const Plane &plane) const;

    void insertPoints(vector<Point3D> points);

    void mergePlane(Plane plane);

    void insertImageCoords(vector<ImageCoords> imageCoordsVec);

    Point3D getCentralPoint() const;

    void setColor(const HSVColor &color);

    const vector<ImageCoords> &getImageCoordsVec() const;

    friend std::ostream &operator<<(std::ostream &stream, const Plane &plane) {
        stream << plane.getA() << " " << plane.getB() << " " << plane.getC() << " " << plane.getD();
        return stream;
    }

    void computeNormalVecDirection();

private:
    double A, B, D, C;
    vector<Point3D> points;
    vector<ImageCoords> imageCoordsVec;
    Vector3d planeNormalVec;
    HSVColor color = HSVColor();
    bool valid = false;

    void computePlaneEquation(const Vector3d &point1, const Vector3d &point2, const Vector3d &point3);
};

#endif /* Plane_h */

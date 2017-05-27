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

    Plane(Vector3d point1, Vector3d point2, Vector3d point3, const ImageCoords &imageCoords);

    Plane(std::array<Eigen::Vector3d, 3>, const Mat &colorImage);

    Plane(Vector3d normalVec, Vector3d point, const vector<Vector3d> &points, const ImageCoords &imageCoords);

    Plane(Vector3d normalVec, double D, vector<Vector3d> points, vector<ImageCoords> imageCoordsVec, HSVColor color);

    Plane(Vector3d normalVec, double D); // added for testing, TODO remove after testing

    double getA() const;

    double getB() const;

    double getC() const;

    double getD() const;

    bool isValid() const;

    double getDistanceFromPoint(Eigen::Vector3d point);

    Vector3d getPlaneNormalVec() const;

    Vector3d computePointOnPlaneFromTwoCoordinate(double firstCoordinate = 0, double secondCoordinate = 0) const;

    const ImageCoords &getImageCoords() const;

    void setImageCoords(const ImageCoords &imageCoords);

    const HSVColor &getColor() const;

    const vector<Vector3d> &getPoints() const;

    void setPoints(const vector<Vector3d> &points);

    unsigned int getNumberOfPoints() const;

    double getAngleBetweenTwoPlanes(const Plane &plane) const;

    void insertPoints(vector<Vector3d> points);

    void mergePlane(Plane plane);

    void insertImageCoords(vector<ImageCoords> imageCoordsVec);

    Vector3d getCentralPoint() const;

    void setColor(const HSVColor &color);

    const vector<ImageCoords> &getImageCoordsVec() const;

    friend std::ostream &operator<<(std::ostream &stream, const Plane &plane) {
        stream << plane.getA() << " " << plane.getB() << " " << plane.getC() << " " << plane.getD();
        return stream;
    }

private:
    double A, B, D, C;
    vector<Vector3d> points;
    vector<ImageCoords> imageCoordsVec;
    Vector3d planeNormalVec;
    HSVColor color = HSVColor();
    bool valid = false;

    void computePlaneEquation(Vector3d point1, Vector3d point2, Vector3d point3);
};

#endif /* Plane_h */

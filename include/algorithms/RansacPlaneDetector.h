//
// Created by mordimer on 12.03.17.
//

#ifndef PROJEKTMAGISTERSKI_PLANERANSAC_H
#define PROJEKTMAGISTERSKI_PLANERANSAC_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "include/models/Plane.h"
#include "include/utils/constants.h"
#include "include/models/PlaneDetector.h"

using namespace Eigen;
using namespace std;

class RansacPlaneDetector : public PlaneDetector {
private:
    static const int MAX_ITERATIONS_NUM = 10000;
    static constexpr double INLAIERS_PERCENT_CONSENSUS = 0.8f;
    static constexpr double MAX_INLARIES_POINT_PLANE_DISTANCE = 0.01f;

    void getRandom3Points(const vector<Point3D> &pointsVector, Vector3d *random3Points);

    Plane computePlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords, bool withAcceptedRange);

public:
    Plane
    getPlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords, const Mat *colorImage = nullptr, bool withAcceptedRange = false);
};


#endif //PROJEKTMAGISTERSKI_PLANERANSAC_H

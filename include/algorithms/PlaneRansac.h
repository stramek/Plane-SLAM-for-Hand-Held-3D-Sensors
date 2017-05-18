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

using namespace Eigen;
using namespace std;

class PlaneRansac {
private:
    static const int MAX_ITERATIONS_NUM = 10000;
    static constexpr double INLAIERS_PERCENT_CONSENSUS = 0.8f;
    static constexpr double MAX_INLARIES_POINT_PLANE_DISTANCE = 1.0f;
    static void getRandom3Points(const vector<Vector3d> &pointsVector, Vector3d *random3Points);
    static Plane computePlane(const vector<Vector3d> &pointsVector, const Mat &colorImage );
public:
    static Plane getPlane(const vector<Vector3d> &pointsVector, const Mat &colorImage );
};


#endif //PROJEKTMAGISTERSKI_PLANERANSAC_H

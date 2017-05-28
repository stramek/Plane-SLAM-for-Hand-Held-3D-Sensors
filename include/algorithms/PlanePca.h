//
// Created by mordimer on 26.02.17.
//

#ifndef PROJEKTMAGISTERSKI_PLANEPCA_H
#define PROJEKTMAGISTERSKI_PLANEPCA_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "include/models/Plane.h"
#include "include/utils/constants.h"
#include "include/models/ImageCoords.h"

using namespace Eigen;
using namespace std;

typedef Matrix<double, 3, 3> Mat33;

class PlanePca {
private:
    static void pointsVectorToMatrix(const vector<Vector3d> &pointsVector, MatrixXf &matrix);

    static Mat33 computeCovMatrix(const vector<Point3D> &pointsVector, const Vector3d &mean);

    static Vector3d computeMean(const vector<Point3D> &pointsVector);

    static Plane computePlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords);

public:
    static Plane getPlane(const vector<Point3D> &pointsVector, const vector<Point3D> &points, const ImageCoords &imageCoords);

    static Plane getPlane(const vector<Point3D> &pointsVector, const Mat& colorImage, const ImageCoords& imageCoords);
};


#endif //PROJEKTMAGISTERSKI_PCA_H

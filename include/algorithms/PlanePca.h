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
#include "include/models/PlaneDetector.h"

using namespace Eigen;
using namespace std;

typedef Matrix<double, 3, 3> Mat33;

class PlanePca : public PlaneDetector{
private:
    void pointsVectorToMatrix(const vector<Vector3d> &pointsVector, MatrixXf &matrix);

    Mat33 computeCovMatrix(const vector<Point3D> &pointsVector, const Vector3d &mean);

    Vector3d computeMean(const vector<Point3D> &pointsVector);

    Plane computePlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords);

public:
    //static Plane getPlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords);

    Plane getPlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords, const Mat *colorImage = nullptr);
};


#endif //PROJEKTMAGISTERSKI_PCA_H

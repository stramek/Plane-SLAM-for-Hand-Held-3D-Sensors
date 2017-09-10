//
// Created by mordimer on 26.02.17.
//

#ifndef PROJEKTMAGISTERSKI_PCAPLANEDETECTOR_H
#define PROJEKTMAGISTERSKI_PCAPLANEDETECTOR_H

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

class PcaPlaneDetector : public PlaneDetector{
private:
    void pointsVectorToMatrix(const vector<Vector3d> &pointsVector, MatrixXf &matrix);

    Mat33 computeCovMatrix(const vector<Point3D> &pointsVector, const Vector3d &mean);

    Plane computePlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords, bool withAcceptedRange = false);

public:
    //static Plane getPlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords);

    Plane getPlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords, const Mat *colorImage = nullptr, bool withAcceptedRange = false);
};


#endif //PROJEKTMAGISTERSKI_PCA_H

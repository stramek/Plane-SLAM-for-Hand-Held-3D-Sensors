//
// Created by mordimer on 26.02.17.
//

#ifndef PROJEKTMAGISTERSKI_PLANEPCA_H
#define PROJEKTMAGISTERSKI_PLANEPCA_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "../models/Plane.h"
#include "../utils/constants.h"
#include "include/models/ImageCoords.h"

using namespace Eigen;
using namespace std;


class PlanePca {
private:
    static void pointsVectorToMatrix(const vector<Vector3f> &pointsVector, MatrixXf &matrix);

    static MatrixXf computeCovMatrix(const MatrixXf &matrix);

    static Plane computePlane(const vector<Vector3f> &pointsVector, const Mat& colorImage, const ImageCoords& imageCoords);

public:
    static Plane getPlane(const vector<Vector3f> &pointsVector, const Mat& colorImage, const ImageCoords& imageCoords);
};


#endif //PROJEKTMAGISTERSKI_PCA_H

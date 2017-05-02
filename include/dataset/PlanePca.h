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

using namespace Eigen;
using namespace std;


class PlanePca {
private:
    static void pointsVectorToMatrix(const vector<Vector3f> &pointsVector, MatrixXf &matrix);

    static MatrixXf computeCovMatrix(const MatrixXf &matrix);

    static Plane computePlane(const vector<Vector3f> &pointsVector, const Mat& colorImage);

public:
    static Plane getPlane(const vector<Vector3f> &pointsVector, const Mat& colorImage);
};


#endif //PROJEKTMAGISTERSKI_PCA_H

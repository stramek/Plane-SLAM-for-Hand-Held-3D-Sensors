//
// Created by mordimer on 26.02.17.
//

#ifndef PROJEKTMAGISTERSKI_PLANEPCA_H
#define PROJEKTMAGISTERSKI_PLANEPCA_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "../models/Plane.h"

using namespace Eigen;
using namespace std;


class PlanePca {
private:
    static void pointsVectorToMatrix(const vector<Vector3f> &pointsVector, MatrixXf &matrix);

    static MatrixXf computeCovMatrix(const MatrixXf &matrix);

    static Plane computePlane(const vector<Vector3f> &pointsVector);

    const static float PCA_MAX_ACCEPTED_DISTANCE = 2.0;
public:
    static Plane getPlane(const vector<Vector3f> &pointsVector);
};


#endif //PROJEKTMAGISTERSKI_PCA_H

//
// Created by mordimer on 26.02.17.
//

#ifndef PROJEKTMAGISTERSKI_PCA_H
#define PROJEKTMAGISTERSKI_PCA_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "../kinect/Plane.h"

using namespace Eigen;
using namespace std;


class PCA {
private:
    static void vecContainerToMatrix(vector<Vector3f> pointsVector, MatrixXf &matrix);

    static MatrixXf computeCovMatrix(MatrixXf matrix);

    static void computePlane(vector<Vector3f> pointsVector);

public:
    static void getPlane(vector<Vector3f> pointsVector);
};


#endif //PROJEKTMAGISTERSKI_PCA_H

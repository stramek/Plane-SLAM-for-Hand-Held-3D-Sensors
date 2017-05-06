//
// Created by stramek on 26.02.17.
//
#ifndef PROJEKTMAGISTERSKI_CONSTANTS_H
#define PROJEKTMAGISTERSKI_CONSTANTS_H

#include <Eigen/Dense>

using namespace Eigen;

const static float PCA_MAX_ACCEPTED_DISTANCE = 2.0f;

const double FOCAL_LENGTH_X = 525;
const double FOCAL_LENGTH_Y = 525;
const double OPTICAL_CENTER_X = 319.5;
const double OPTICAL_CENTER_Y = 239.5;

const int MAX_SIMILARITY_VALUE = 5;
const int MAX_ANGLE_BETWEEN_PLANES = 15;

const Matrix<double, 3, 3> PHCP_MODEL = [] {
    Matrix<double, 3, 3> matrix;
    matrix << 1 / FOCAL_LENGTH_X, 0, -OPTICAL_CENTER_X / FOCAL_LENGTH_X,
            0, 1 / FOCAL_LENGTH_Y, -OPTICAL_CENTER_Y / FOCAL_LENGTH_Y,
            0, 0, 1;
    return matrix;
}();

#endif //PROJEKTMAGISTERSKI_CONSTANTS_H
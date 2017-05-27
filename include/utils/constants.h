//
// Created by stramek on 26.02.17.
//
#ifndef PROJEKTMAGISTERSKI_CONSTANTS_H
#define PROJEKTMAGISTERSKI_CONSTANTS_H

#include <Eigen/Dense>

using namespace Eigen;

const static float PCA_MAX_ACCEPTED_DISTANCE = 0.010f;
const static float CLUSTERING_MAX_ANGLE_THRESHOLD = 10.0;
const static double CLUSTERING_MAX_DISTANCE_THRESHOLD = 0.5;
const int MAX_SIMILARITY_VALUE = 10;
const int MAX_ANGLE_BETWEEN_PLANES = 20;

const float MAX_PERCENT_OF_NAN_PIXELS_INSIDE_PLAIN = 10.0f;

const double FOCAL_LENGTH_X = 525;
const double FOCAL_LENGTH_Y = 525;
const double OPTICAL_CENTER_X = 319.5;

const double OPTICAL_CENTER_Y = 239.5;

const bool DEBUG = false;

enum KINECT_STATES {
    VISUALIZE_SIMILAR_PLANES, SHOW_POINTCLOUD, SLAM
};

const KINECT_STATES KINECT_MODE = VISUALIZE_SIMILAR_PLANES;

const Matrix<double, 3, 3> PHCP_MODEL = [] {
    Matrix<double, 3, 3> matrix;
    matrix << 1 / FOCAL_LENGTH_X, 0, -OPTICAL_CENTER_X / FOCAL_LENGTH_X,
            0, 1 / FOCAL_LENGTH_Y, -OPTICAL_CENTER_Y / FOCAL_LENGTH_Y,
            0, 0, 1;
    return matrix;
}();

static int SCREENSHOT_HELPER = 1;

#endif //PROJEKTMAGISTERSKI_CONSTANTS_H
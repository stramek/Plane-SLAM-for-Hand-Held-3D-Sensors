//
// Created by stramek on 09.03.17.
//
#ifndef PROJEKTMAGISTERSKI_UTILS_H
#define PROJEKTMAGISTERSKI_UTILS_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "include/models/Plane.h"
#include "include/utils/PlaneSimilarity.h"

using namespace cv;
using namespace std;
using namespace Eigen;

namespace utils {
    void paintPixel(Mat &rgb, const Vector3f &vector, Vec3b color);
    pair<int, int> getRandomPosition(const Mat &mat, int areaSize);
    vector<pair<Plane, Plane>> getSimilarPlanes(const vector<Plane> &previousFrame, const vector<Plane> &currentFrame);
}

#endif //PROJEKTMAGISTERSKI_UTILS_H
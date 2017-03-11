//
// Created by stramek on 09.03.17.
//
#ifndef PROJEKTMAGISTERSKI_UTILS_H
#define PROJEKTMAGISTERSKI_UTILS_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace Eigen;

namespace utils {
    void paintPixel(Mat &rgb, const Vector3f &vector, Vec3b color);
    pair<int, int> getRandomPosition(const Mat &mat, int areaSize);
}

#endif //PROJEKTMAGISTERSKI_UTILS_H
//
// Created by stramek on 09.03.17.
//
#ifndef PROJEKTMAGISTERSKI_UTILS_H
#define PROJEKTMAGISTERSKI_UTILS_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "include/models/Plane.h"
#include "include/models/PlaneSimilarity.h"
#include <include/algorithms/PlanePca.h>
#include "include/models/ImageCoords.h"
#include <include/models/ImagePair.h>
#include <limits.h>
#include <algorithm>
#include <random>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace octomap;

namespace utils {
    void paintPixel(Mat &rgb, const Vector3f &vector, Vec3b color);
    pair<int, int> getRandomPosition(const Mat &mat, int areaSize);
    pair<int, int> getRandomPosition(const int cols, const int rows, int areaSize);
    void generateOctoMap(const string fileName, const vector<Point3D> pointCloud, const float resolution = 0.05);
}

#endif //PROJEKTMAGISTERSKI_UTILS_H
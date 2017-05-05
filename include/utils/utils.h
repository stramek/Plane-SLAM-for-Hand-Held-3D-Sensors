//
// Created by stramek on 09.03.17.
//
#ifndef PROJEKTMAGISTERSKI_UTILS_H
#define PROJEKTMAGISTERSKI_UTILS_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "include/models/Plane.h"
#include "include/models/PlaneSimilarity.h"
#include <include/dataset/PlanePca.h>
#include "include/models/ImageCoords.h"
#include <include/dataset/ImagePair.h>
#include <limits.h>
#include <algorithm>
#include <random>

using namespace cv;
using namespace std;
using namespace Eigen;

namespace utils {
    void paintPixel(Mat &rgb, const Vector3f &vector, Vec3b color);
    pair<int, int> getRandomPosition(const Mat &mat, int areaSize);
    vector<pair<Plane, Plane>> getSimilarPlanes(const vector<Plane> &previousFrame, const vector<Plane> &currentFrame);
    void fillPlaneVector(int numberOfPoints, int areaSize, ImagePair &imagePair,
                         vector<Plane> *planeVector, vector<Plane> *previousPlaneVector = nullptr,
                         float previousPlanePercent = 0.5, bool colorPlanes = false);
    void visualizeSimilarPlanes(vector<pair<Plane, Plane>> &similarPlanes, const Mat &previousImage,
                                const Mat &currentImage, int limitPoints = INT_MAX);
    void filterPairsByAngle(vector<pair<Plane, Plane>> &pairs);
}

#endif //PROJEKTMAGISTERSKI_UTILS_H
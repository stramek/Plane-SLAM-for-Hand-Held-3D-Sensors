//
// Created by stramek on 05.05.17.
//
#ifndef PROJEKTMAGISTERSKI_PLANEUTILS_H
#define PROJEKTMAGISTERSKI_PLANEUTILS_H

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
#include "include/utils/utils.h"
#include "include/algorithms/Clustering.h"

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace octomap;

namespace planeUtils {
    vector<pair<Plane, Plane>> getSimilarPlanes(const vector<Plane> &previousFrame, const vector<Plane> &currentFrame);
    void fillPlaneVector(int numberOfPoints, int areaSize, ImagePair &imagePair,
                         vector<Plane> *planeVector, vector<Plane> *previousPlaneVector = nullptr,
                         float previousPlanePercent = 0.5, bool colorPlanes = false);
    void visualizeSimilarPlanes(vector<pair<Plane, Plane>> &similarPlanes, const Mat &previousImage,
                                const Mat &currentImage, int limitPoints = INT_MAX);
    void filterPairsByAngle(vector<pair<Plane, Plane>> &pairs);
    void mergePlanes(vector<Plane> &planeVector);
    void displayClusteredPlanes(ImagePair &imagePair, vector<Plane> plane);
}

#endif //PROJEKTMAGISTERSKI_PLANEUTILS_H
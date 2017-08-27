//
// Created by stramek on 09.03.17.
//
#ifndef PROJEKTMAGISTERSKI_UTILS_H
#define PROJEKTMAGISTERSKI_UTILS_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "include/models/Plane.h"
#include "include/models/PlaneSimilarity.h"
#include <include/algorithms/PcaPlaneDetector.h>
#include "include/models/ImageCoords.h"
#include <include/models/ImagePair.h>
#include <limits.h>
#include <algorithm>
#include <random>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <boost/algorithm/string.hpp>
#include "include/models/PosOrient.h"
#include "boost/lexical_cast.hpp"

namespace utils {
    void paintPixel(cv::Mat &rgb, const Eigen::Vector3f &vector, cv::Vec3b &color);
    pair<int, int> getRandomPosition(const cv::Mat &mat, int areaSize);
    pair<int, int> getRandomPosition(const int cols, const int rows, int areaSize);
    void generateOctoMap(const std::string fileName, const std::vector<Point3D> pointCloud, const float resolution = 0.05);
    void loadDatasetPositions(vector<PosOrient> &positions);
    void movePlanesToPreviousVector(vector<Plane> &planeVectorPreviousFrame, vector<Plane> &planeVectorCurrentFrame);
    string getCurrentDate();
}

#endif //PROJEKTMAGISTERSKI_UTILS_H
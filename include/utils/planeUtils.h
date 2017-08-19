//
// Created by stramek on 05.05.17.
//
#ifndef PROJEKTMAGISTERSKI_PLANEUTILS_H
#define PROJEKTMAGISTERSKI_PLANEUTILS_H

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
#include "include/utils/utils.h"
#include "include/algorithms/Clustering.h"
#include <include/models/PointCloud.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <include/algorithms/RansacPlaneDetector.h>
#include <iomanip>
#include <sstream>
#include "include/utils/constants.h"

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace octomap;

namespace planeUtils {
    vector<pair<Plane, Plane>> getSimilarPlanes(vector<Plane> &previousFrame, vector<Plane> &currentFrame);
    void fillPlaneVector(int numberOfPoints, int areaSize, ImagePair &imagePair,
                         vector<Plane> *planeVector, vector<Plane> *previousPlaneVector = nullptr,
                         double previousPlanePercent = 0.5, bool colorPlanes = true);
    void fillPlaneVector(int numberOfPoints, int areaSize, vector<Plane> *planeVector, vector<Plane> *previousPlaneVector, double previousPlanePercent,
                         libfreenect2::Registration *registration, libfreenect2::Frame *undistorted, libfreenect2::Frame *registered);
    void visualizeSimilarPlanes(vector<pair<Plane, Plane>> &similarPlanes, const Mat &previousImage,
                                const Mat &currentImage, int limitPoints = INT_MAX);
    void mergePlanes(vector<Plane> &planeVector, PlaneDetector *planeDetector);
    void displayClusteredPlanes(ImagePair &imagePair, vector<Plane> planes);
    Mat getRGBFrameMat(libfreenect2::Registration *registration, libfreenect2::Frame *undistorted, libfreenect2::Frame *registered);
    double getDistanceBetweenTwoPlanes(const Plane &firstPlane, const Plane &secondPlane);
    bool arePlanesValid(vector<Plane> &planes);
    bool arePlanesValid(const Plane &plane1, const Plane &plane2, const Plane &plane3);
}

#endif //PROJEKTMAGISTERSKI_PLANEUTILS_H
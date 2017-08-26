//
// Created by mordimer on 17.08.17.
//

#include "include/planeG2O/MatchPlanesG2o.h"
using namespace Eigen;

vector<pair<Plane, Plane>> MatchPlanesG2o::getSimilarPlanes(vector<Plane> &previousFrame, vector<Plane> &currentFrame){
    computePreviousFramePlanesIndexes(previousFrame);
    computeCurrentFramePlanesIndexes(currentFrame);

    PlaneG2oModule planeG2o;
    PosOrient posOrient;
    int counter = 0;
    for (auto i : previousFramePlanesIndexes) {
        if (planeUtils::arePlanesValid(previousFrame.at(i[0]), previousFrame.at(i[1]), previousFrame.at(i[2]))) {
            for (auto j : currentFramePlanesIndexes) {
                if (planeUtils::arePlanesValid(currentFrame.at(j[0]), currentFrame.at(j[1]),
                                               currentFrame.at(j[2]))) {
                    vector<pair<Plane, Plane>> matchedPlanes;
                    for (int k = 0; k < 3; ++k) {
                        pair<Plane, Plane> planesPair(previousFrame.at(i[k]), currentFrame.at(j[k]));
                        matchedPlanes.push_back(planesPair);
                    }
                    cout << counter << endl;
                    counter++;
                    posOrient = planeG2o.ComputeCameraPos(matchedPlanes);
                    if (validateMatch(matchedPlanes, posOrient)) {
                        matchRemainingPlanes(matchedPlanes, i, j, previousFrame, currentFrame, posOrient);
                        return matchedPlanes;
                    }
                }
            }
        }
    }
    cout << "Matched failed!" << endl;
    vector<pair<Plane, Plane>> matchedPlanes;
    return matchedPlanes;
}

void MatchPlanesG2o::computePreviousFramePlanesIndexes(vector<Plane> &previousFrame){
    previousFramePlanesIndexes.clear();
    for(int firstIndex = 0; firstIndex < previousFrame.size(); ++firstIndex){
        for(int secondIndex = firstIndex + 1; secondIndex < previousFrame.size(); ++secondIndex){
            for(int thirdIndex = secondIndex + 1; thirdIndex < previousFrame.size(); ++thirdIndex){
                previousFramePlanesIndexes.push_back(Vector3i(firstIndex, secondIndex, thirdIndex));
            }
        }
    }
}

void MatchPlanesG2o::computeCurrentFramePlanesIndexes(vector<Plane> &currentFrame){
    currentFramePlanesIndexes.clear();
    for(int firstIndex = 0; firstIndex < currentFrame.size(); ++firstIndex){
        for(int secondIndex = 0; secondIndex < currentFrame.size(); ++secondIndex){
            for(int thirdIndex = 0; thirdIndex < currentFrame.size(); ++thirdIndex){
                if (firstIndex != secondIndex && firstIndex != thirdIndex && secondIndex != thirdIndex)
                    currentFramePlanesIndexes.push_back(Vector3i(firstIndex, secondIndex, thirdIndex));
            }
        }
    }
}

bool MatchPlanesG2o::validateMatch(vector<pair<Plane, Plane>> &matchedPlanes, PosOrient &posOrient){
    Quaterniond q = posOrient.getQuaternion();
    Vector3d position = posOrient.getPosition();
    Vector3d eulerAngles = fromRotationMat(q.toRotationMatrix());
    if (abs(eulerAngles(0)) > MaxRotTransformation && abs(eulerAngles(1)) > MaxRotTransformation &&
            abs(eulerAngles(2)) > MaxRotTransformation)
        return false;

    if (position.norm() > MaxPosTransformation)
        return false;

    for(auto &pair : matchedPlanes) {
        Vector3d vectorAfterRot;
        vectorAfterRot = q.toRotationMatrix() * pair.first.getPlaneNormalVec();
        cout<< "Angle between vectors: " << getAngleBetweenTwoVectors(vectorAfterRot, pair.second.getPlaneNormalVec()) << endl;
        if (getAngleBetweenTwoVectors(vectorAfterRot, pair.second.getPlaneNormalVec()) >  MaxAngleDiffrence)
            return false;
    }
    return true;
}

double MatchPlanesG2o::getAngleBetweenTwoVectors(const Vector3d &v1, const Vector3d &v2) {

    double angleCos =  v1.dot(v2)/ v1.norm() / v2.norm();
    if (angleCos < -1) angleCos = -1.0;
    if (angleCos > 1) angleCos = 1.0;
    double angle = acos(angleCos) * 180.0 / (double) M_PI;

    return angle;
}

/// compute roll/pitch/yaw from rotation matrix
Vector3d MatchPlanesG2o::fromRotationMat(const Mat33& pose){
    Vector3d rpy(Vector3d::Identity());
    rpy.x() = atan2(pose(2,1), pose(2,2)) * 180.0 / (double) M_PI;
    rpy.y() = -asin(pose(2,0)) * 180.0 / (double) M_PI;
    rpy.z() = atan2(pose(1,0), pose(0,0)) * 180.0 / (double) M_PI;
    return rpy;
}

void MatchPlanesG2o::matchRemainingPlanes(vector<pair<Plane, Plane>> &matchedPlanes, Eigen::Vector3i &matchedIndexesPrevPlane,
                                          Eigen::Vector3i &matchedIndexesCurPlane, const vector<Plane> &previousFrame,
                                          const vector<Plane> &currentFrame, const PosOrient &posOrient){
    int prevPlaneIterCounter = 0;
    for (auto prevPlane : previousFrame) {
        int curPlaneIterCounter = 0;
        for (auto curPlane : currentFrame) {
            if (matchedIndexesPrevPlane(0) != prevPlaneIterCounter && matchedIndexesPrevPlane(1) != prevPlaneIterCounter &&
                matchedIndexesPrevPlane(2) != prevPlaneIterCounter) {
                if (matchedIndexesCurPlane(0) != curPlaneIterCounter && matchedIndexesCurPlane(1) != curPlaneIterCounter &&
                    matchedIndexesCurPlane(2) != curPlaneIterCounter) {
                    if (getAngleBetweenTwoVectors(prevPlane.getPlaneNormalVec(), curPlane.getPlaneNormalVec()) <  MaxAngleDiffrence) {
                        if (planeUtils::getDistanceBetweenTwoPlanes(prevPlane, curPlane) < CLUSTERING_MAX_DISTANCE_THRESHOLD) {
                            pair<Plane, Plane> planesPair(prevPlane, curPlane);
                            matchedPlanes.push_back(planesPair);
                        }
                    }
                }
            }
            ++curPlaneIterCounter;
        }
        ++prevPlaneIterCounter;
    }
}
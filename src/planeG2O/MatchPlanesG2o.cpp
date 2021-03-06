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
                if (planeUtils::arePlanesValid(currentFrame.at(j[0]), currentFrame.at(j[1]), currentFrame.at(j[2])) &&
                    planeUtils::arePlanesValid(previousFrame.at(i[0]), previousFrame.at(i[1]), previousFrame.at(i[2]))) {
                    vector<pair<Plane, Plane>> matchedPlanes;
                    for (int k = 0; k < 3; ++k) {
                        pair<Plane, Plane> planesPair(previousFrame.at(i[k]), currentFrame.at(j[k]));
                        matchedPlanes.push_back(planesPair);
                    }
                    if(areColorValid(matchedPlanes)){
                        posOrient = planeG2o.ComputeCameraPos(matchedPlanes);
                        if (validateMatch(matchedPlanes, posOrient)) {
                            resetWasMatch(previousFrame);
                            currentFrame.at(j(0)).setWasMatched(true);
                            currentFrame.at(j(1)).setWasMatched(true);
                            currentFrame.at(j(2)).setWasMatched(true);
                            previousFrame.at(i(0)).setWasMatched(true);
                            previousFrame.at(i(1)).setWasMatched(true);
                            previousFrame.at(i(2)).setWasMatched(true);
                            matchRemainingPlanes(matchedPlanes, i, j, previousFrame, currentFrame, posOrient);
                            unmatchedPlanes.clear();
                            for (auto &plane : currentFrame) {
                                if (!plane.isWasMatched()){
                                    unmatchedPlanes.push_back(plane);
                                }
                            }
                            return matchedPlanes;
                        }
                    }
                }
            }
        }
    }
/*    unmatchedPlanes.clear();
    for (auto &plane : currentFrame) {
        if (!plane.isWasMatched()){
            unmatchedPlanes.push_back(plane);
        }
    }*/
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
    pair<Vector3d, Vector3d> diff = posOrient.minus(lastPosOrient);
    Vector3d position = diff.first;
    Vector3d angles = diff.second;
//    std::cout << "-----------------------------------------------" <<std::endl;
//    std::cout << "Position diff: " << position[0] << " " << position[1] << " " << position[2] << std::endl;
//    std::cout << "Angle diff: " << angles[0] << " " << angles[1] << " " << angles[2] << std::endl;
    return !(position[0] > G2O_MAX_DISPLACEMENT || position[1] > G2O_MAX_DISPLACEMENT ||
             position[2] > G2O_MAX_DISPLACEMENT || angles[0] > G2O_MAX_ROTATION ||
             angles[1] > G2O_MAX_ROTATION || angles[2] > G2O_MAX_ROTATION);

}

double MatchPlanesG2o::getAngleBetweenTwoVectors(const Vector3d &v1, const Vector3d &v2) {

    double angleCos =  v1.dot(v2)/ v1.norm() / v2.norm();
    if (angleCos < -1) angleCos = -1.0;
    if (angleCos > 1) angleCos = 1.0;
    double angle = acos(angleCos) * 180.0 / (double) M_PI;

    return angle;
}

void MatchPlanesG2o::matchRemainingPlanes(vector<pair<Plane, Plane>> &matchedPlanes, Eigen::Vector3i &matchedIndexesPrevPlane,
                                          Eigen::Vector3i &matchedIndexesCurPlane,  vector<Plane> &previousFrame,
                                          vector<Plane> &currentFrame, PosOrient &posOrient){
    int prevPlaneIterCounter = 0;
    for (auto &prevPlane : previousFrame) {
        int curPlaneIterCounter = 0;
        for (auto &curPlane : currentFrame) {
            if (matchedIndexesCurPlane(0) != curPlaneIterCounter && matchedIndexesCurPlane(1) != curPlaneIterCounter &&
                matchedIndexesCurPlane(2) != curPlaneIterCounter) {
                if (matchedIndexesPrevPlane(0) != prevPlaneIterCounter && matchedIndexesPrevPlane(1) != prevPlaneIterCounter &&
                    matchedIndexesPrevPlane(2) != prevPlaneIterCounter) {
                    Plane planeConverted = curPlane.getPlaneSeenFromGlobalCamera(posOrient);
                    if (abs(prevPlane.getD() - planeConverted.getD()) < D_THRESHOLD){
                        if (getAngleBetweenTwoVectors(prevPlane.getPlaneNormalVec(), planeConverted.getPlaneNormalVec()) <  MAX_ANGLE_BETWEEN_PLANES_GLOBAL_MAP) {
                            if (abs(prevPlane.getColor().getHue() - curPlane.getColor().getHue()) < MAX_SIMILARITY_GLOBAL_MAP_VALUE) {
                                if(!curPlane.isWasMatched() && !prevPlane.isWasMatched()){
                                    curPlane.setWasMatched(true);
                                    prevPlane.setWasMatched(true);
                                    pair<Plane, Plane> planesPair(prevPlane, curPlane);
                                    matchedPlanes.push_back(planesPair);
                                }
                            }
                        }
                    }
                }
            }
            ++curPlaneIterCounter;
        }
        ++prevPlaneIterCounter;
    }
}

void MatchPlanesG2o::setLastPosOrient(const PosOrient &posOrient) {
    MatchPlanesG2o::lastPosOrient = posOrient;
}

vector<Plane> &MatchPlanesG2o::getUnmatchedPlanes() {
    return unmatchedPlanes;
}

bool MatchPlanesG2o::areColorValid(vector<pair<Plane, Plane>> &matchedPlanes) {
    for(auto &planePair : matchedPlanes) {
        if(abs(planePair.first.getColor().getHue() - planePair.second.getColor().getHue()) > MAX_SIMILARITY_GLOBAL_MAP_VALUE)
            return false;
    }
    return true;
}

void MatchPlanesG2o::resetWasMatch(vector<Plane> &planes){
    for(auto &plane : planes) {
        plane.setWasMatched(false);
    }
}


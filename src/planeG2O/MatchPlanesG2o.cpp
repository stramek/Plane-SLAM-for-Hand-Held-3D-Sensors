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
                if (planeUtils::arePlanesValid(previousFrame.at(j[0]), previousFrame.at(j[1]), previousFrame.at(j[2]))) {
                    vector<pair<Plane, Plane>> matchedPlanes;
                    for (int k = 0; k < 3; ++k) {
                        pair<Plane, Plane> planesPair(previousFrame.at(i[k]), currentFrame.at(j[k]));
                        matchedPlanes.push_back(planesPair);
                    }
                    cout << counter << endl;
                    counter++;
                    posOrient = planeG2o.ComputeCameraPos(matchedPlanes);
                    if (validateMatch(matchedPlanes, posOrient)) {
                        cout << "Matched succes!" << endl;
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
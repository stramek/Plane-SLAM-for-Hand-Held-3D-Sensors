//
// Created by mordimer on 17.08.17.
//

#include "include/planeG2O/MatchPlanesG2o.h"
using namespace Eigen;

vector<pair<Plane, Plane>> MatchPlanesG2o::getSimilarPlanes(vector<Plane> &previousFrame, vector<Plane> &currentFrame){
    computePreviousFramePlanesIndexes(previousFrame);
    computeCurrentFramePlanesIndexes(currentFrame);

    for (int i = 0; i < previousFramePlanesIndexes.size(); ++i) {
        for (int j = 0; j < currentFramePlanesIndexes.size(); ++j) {

        }
    }
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
//
// Created by mordimer on 17.08.17.
//

#ifndef PROJEKTMAGISTERSKI_MATCHPLANESG2O_H
#define PROJEKTMAGISTERSKI_MATCHPLANESG2O_H


#include <include/models/Plane.h>
#include <Eigen/Dense>
#include "include/utils/planeUtils.h"
#include "include/planeG2O/PlaneG2oModule.h"

class MatchPlanesG2o {
public:
    vector<pair<Plane, Plane>> getSimilarPlanes(vector<Plane> &previousFrame, vector<Plane> &currentFrame);

private:
    vector<Eigen::Vector3i> previousFramePlanesIndexes;
    vector<Eigen::Vector3i> currentFramePlanesIndexes;

    void computePreviousFramePlanesIndexes(vector<Plane> &previousFrame);
    void computeCurrentFramePlanesIndexes(vector<Plane> &currentFrame);
    bool validateMatch(vector<pair<Plane, Plane>> &matchedPlanes, PosOrient &posOrient);
    double getAngleBetweenTwoVectors(const Vector3d &v1, const Vector3d &v2);
    const double MaxAngleDiffrence = 5.0;
};


#endif //PROJEKTMAGISTERSKI_MATCHPLANESG2O_H

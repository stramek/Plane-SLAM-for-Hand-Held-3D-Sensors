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

    vector<Plane> unmatchedPlanes;
public:
    vector<Plane> &getUnmatchedPlanes();
    bool areColorValid(vector<pair<Plane, Plane>> &matchedPlanes);

private:

    void computePreviousFramePlanesIndexes(vector<Plane> &previousFrame);
    void computeCurrentFramePlanesIndexes(vector<Plane> &currentFrame);
    bool validateMatch(vector<pair<Plane, Plane>> &matchedPlanes, PosOrient &posOrient);
    double getAngleBetweenTwoVectors(const Vector3d &v1, const Vector3d &v2);
    void matchRemainingPlanes(vector<pair<Plane, Plane>> &matchedPlanes, Eigen::Vector3i &matchedIndexesPrevPlane,
                              Eigen::Vector3i &matchedIndexesCurPlane, vector<Plane> &previousFrame,
                              vector<Plane> &currentFrame, PosOrient &posOrient);

    PosOrient lastPosOrient = PosOrient(Eigen::Vector3d(0, 0, 0), Eigen::Vector4d(0, 0, 0, 1));

    void resetWasMatch(vector<Plane> &planes);
public:
    void setLastPosOrient(const PosOrient &posOrient);
};


#endif //PROJEKTMAGISTERSKI_MATCHPLANESG2O_H

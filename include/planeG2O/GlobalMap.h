//
// Created by stramek on 24.08.17.
//

#ifndef PROJEKTMAGISTERSKI_GLOBALMAP_H
#define PROJEKTMAGISTERSKI_GLOBALMAP_H

#include "include/models/Plane.h"
#include "include/utils/planeUtils.h"
#include "unordered_map"

class GlobalMap {
public:
    static GlobalMap &getInstance();

    pair<long, bool> addPlaneToMap(Plane &plane, PosOrient &posOrient);

    const unordered_map<long, Plane> &getGlobalMapPlanes() const;

    vector<Plane> getGlobalMapVector() const;

    void updatePlane(Plane &plane);

    Plane getPlaneById(long id);

    pair<long, bool> getIdByPlane(Plane &plane);

    long getCurrentId() const;

private:
    GlobalMap();

    ~GlobalMap(){};

    long currentId = -1;

    std::unordered_map<long, Plane> globalMapPlanes;

    bool isSimilarPlaneExists(Plane &plane);

    bool isAngleBetweenPlanesValid(Plane &plane1, Plane &plane2);

    bool isDistanceBetweenPlanesValid(Plane &plane1, Plane &plane2);

    bool isHueDiffValid(Plane &plane1, Plane &plane2);

    void assignIdToPlane(Plane &plane);
};

#endif //PROJEKTMAGISTERSKI_GLOBALMAP_H

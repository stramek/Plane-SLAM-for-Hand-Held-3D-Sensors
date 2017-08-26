//
// Created by stramek on 24.08.17.
//
#include "include/planeG2O/GlobalMap.h"

GlobalMap &GlobalMap::getInstance() {
    static GlobalMap instance;
    return instance;
}

pair<long, bool> GlobalMap::addPlaneToMap(Plane &plane, PosOrient &posOrient) {
    pair<long, bool> foundId = getIdByPlane(plane);
    if (!foundId.second) {
        Plane transformedPlane = plane.getPlaneSeenFromGlobalCamera(posOrient);
        assignIdToPlane(transformedPlane);
        globalMapPlanes.insert(pair<long, Plane>(transformedPlane.getId(), transformedPlane));
        return pair<long, bool>(currentId, true);
    } else {
        return pair<long, bool>(foundId.first, false);
    }
}

Plane GlobalMap::getPlaneById(long id) {
    return getGlobalMapPlanes().at(id);
}

bool GlobalMap::isSimilarPlaneExists(Plane &plane) {
    for (auto &mapPlane : globalMapPlanes) {
        if (isAngleBetweenPlanesValid(plane, mapPlane.second)
            && isDistanceBetweenPlanesValid(plane, mapPlane.second)
            && isHueDiffValid(plane, mapPlane.second)) {
            return true;
        }
    }
    return false;
}

pair<long, bool> GlobalMap::getIdByPlane(Plane &plane) {
    for (auto &mapPlane : globalMapPlanes) {
        if (isAngleBetweenPlanesValid(plane, mapPlane.second)
            && isDistanceBetweenPlanesValid(plane, mapPlane.second)
            && isHueDiffValid(plane, mapPlane.second)) {
            return pair<long, bool>(mapPlane.second.getId(), true);
        }
    }
    return pair<long, bool>(-1, false);
}

bool GlobalMap::isAngleBetweenPlanesValid(Plane &plane1, Plane &plane2) {
    return plane1.getAngleBetweenTwoPlanes(plane2) < MAX_ANGLE_BETWEEN_PLANES_GLOBAL_MAP;
}

bool GlobalMap::isDistanceBetweenPlanesValid(Plane &plane1, Plane &plane2) {
    return planeUtils::getDistanceBetweenTwoPlanes(plane1, plane2) < MAX_DISTANCE_BETWEEN_PLANES_GLOBAL_MAP;
}

bool GlobalMap::isHueDiffValid(Plane &plane1, Plane &plane2) {
    return abs(plane1.getColor().getHue() - plane2.getColor().getHue()) < MAX_SIMILARITY_GLOBAL_MAP_VALUE;
}

void GlobalMap::assignIdToPlane(Plane &plane) {
    plane.setId(++currentId);
}

void GlobalMap::updatePlane(Plane &plane) {
    if (plane.getId() == -1) throw runtime_error("MICHAU NIE PRZYDZIELIŁ ID XDDDDDDD");
    globalMapPlanes.at(plane.getId()).updatePlaneParameters(plane);
}

const unordered_map<long, Plane> &GlobalMap::getGlobalMapPlanes() const {
    return globalMapPlanes;
}

vector<Plane> GlobalMap::getGlobalMapVector() const {
    vector<Plane> planeVector;
    for (auto &mapPlane : globalMapPlanes) {
        planeVector.push_back(mapPlane.second);
    }
    return planeVector;
}

GlobalMap::GlobalMap() {

}

long GlobalMap::getCurrentId() const {
    return currentId;
}

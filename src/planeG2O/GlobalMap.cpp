//
// Created by stramek on 24.08.17.
//
#include "include/planeG2O/GlobalMap.h"

GlobalMap &GlobalMap::getInstance() {
    static GlobalMap instance;
    return instance;
}

long GlobalMap::addPlaneToMap(Plane &plane, PosOrient &posOrient) {
    Plane transformedPlane = plane.getPlaneSeenFromGlobalCamera(posOrient);
    if (!isSimilarPlaneExists(transformedPlane)) {
        assignIdToPlane(transformedPlane);
        globalMapPlanes.insert(pair<long, Plane>(transformedPlane.getId(), transformedPlane));
        return currentId;
    } else {
        return numeric_limits<long>::quiet_NaN();
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

GlobalMap::GlobalMap() {

}

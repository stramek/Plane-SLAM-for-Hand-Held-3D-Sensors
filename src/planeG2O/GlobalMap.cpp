//
// Created by stramek on 24.08.17.
//
#include "include/planeG2O/GlobalMap.h"

GlobalMap &GlobalMap::getInstance() {
    static GlobalMap instance;
    return instance;
}

tuple<long, bool, Plane> GlobalMap::addPlaneToMap(Plane &plane, PosOrient &posOrient) {
    Plane transformedPlane = plane.getPlaneSeenFromGlobalCamera(posOrient);
    long foundId = getIdByPlane(transformedPlane);
    if (foundId == -1) {
        assignIdToPlane(transformedPlane);
        globalMapPlanes.insert(pair<long, Plane>(transformedPlane.getId(), transformedPlane));
        cout<<"New plane detected! Assigned id = "<<currentId<<endl;
        return tuple<long, bool, Plane>(currentId, true, transformedPlane);
    } else {
        cout<<"Plane already exists! id: "<<foundId<<endl;
        return tuple<long, bool, Plane>(foundId, false, Plane());
    }
}

Plane GlobalMap::getPlaneById(long id) {
    return getGlobalMapPlanes().at(id);
}

long GlobalMap::getIdByPlane(Plane &plane) {
    for (auto &mapPlane : globalMapPlanes) {
        if (isAngleBetweenPlanesValid(plane, mapPlane.second)
            && isDistanceBetweenPlanesValid(plane, mapPlane.second)
            && isHueDiffValid(plane, mapPlane.second)) {
            return mapPlane.second.getId();
        }
    }
    return -1;
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

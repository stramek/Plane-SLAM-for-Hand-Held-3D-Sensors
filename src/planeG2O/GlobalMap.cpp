//
// Created by stramek on 24.08.17.
//
#include <include/models/ImageLoader.h>
#include "include/planeG2O/GlobalMap.h"

GlobalMap &GlobalMap::getInstance() {
    static GlobalMap instance;
    return instance;
}

tuple<long, bool, Plane> GlobalMap::addPlaneToMap(Plane &plane, PosOrient &posOrient, int positionNumber) {
    Plane transformedPlane = plane.getPlaneSeenFromGlobalCamera(posOrient);


    assignIdToPlane(transformedPlane);

    long id = getIdByPlane(transformedPlane);
    if (id == -1) {
        globalMapPlanes.insert(pair<long, Plane>(transformedPlane.getId(), transformedPlane));
    }

    return tuple<long, bool, Plane>(transformedPlane.getId(), id == -1, transformedPlane);

}


Plane GlobalMap::getPlaneById(long id) {
    return getGlobalMapPlanes().at(id);
}

long GlobalMap::getIdByPlane(Plane &plane) {
//    cout<<"***************************************"<<endl;
    for (auto &mapPlane : globalMapPlanes) {
        if (isAngleBetweenPlanesValid(plane, mapPlane.second)
            /*&& isDistanceBetweenPlanesValid(plane, mapPlane.second)*/
            && isHueDiffValid(plane, mapPlane.second)) {
//            cout<<"PLANE FOUND"<<endl;
            return mapPlane.second.getId();
        }
    }
//    cout<<"PLANE NOT FOUND"<<endl;
//    cout<<"***************************************"<<endl;
    return -1;
}

bool GlobalMap::isAngleBetweenPlanesValid(Plane &plane1, Plane &plane2) {
    bool isValid = plane1.getAngleBetweenTwoPlanes(plane2) < MAX_ANGLE_BETWEEN_PLANES_GLOBAL_MAP;
    if (isValid) {
//        cout<<"Plane ANGLE is valid, value: " <<plane1.getAngleBetweenTwoPlanes(plane2)<<endl;
    } else {
//        cout<<"Plane ANGLE is NOT valid, value: " <<plane1.getAngleBetweenTwoPlanes(plane2)<<endl;
    }
    return isValid;
}

bool GlobalMap::isDistanceBetweenPlanesValid(Plane &plane1, Plane &plane2) {
    bool isValid = planeUtils::getDistanceBetweenTwoPlanes(plane1, plane2) < MAX_DISTANCE_BETWEEN_PLANES_GLOBAL_MAP;
    if (isValid) {
//        cout<<"Plane DISTANCE is valid, value: " <<planeUtils::getDistanceBetweenTwoPlanes(plane1, plane2)<<endl;
    } else {
//        cout<<"Plane DISTANCE is NOT valid, value: " <<planeUtils::getDistanceBetweenTwoPlanes(plane1, plane2)<<endl;
    }
    return isValid;
}

bool GlobalMap::isHueDiffValid(Plane &plane1, Plane &plane2) {
    bool isValid = abs(plane1.getColor().getHue() - plane2.getColor().getHue()) < MAX_SIMILARITY_GLOBAL_MAP_VALUE;
    if (isValid) {
//        cout<<"Plane COLOR is valid, value: " <<abs(plane1.getColor().getHue() - plane2.getColor().getHue())<<endl;
    } else {
//        cout<<"Plane COLOR is NOT valid, value: " <<abs(plane1.getColor().getHue() - plane2.getColor().getHue())<<endl;
    }
    return isValid;
}

void GlobalMap::assignIdToPlane(Plane &plane) {
    plane.setId(++currentId);
}

void GlobalMap::updatePlane(Plane &plane) {
    if (plane.getId() == -1) throw runtime_error("MICHAU NIE PRZYDZIELIŁ ID XDDDDDDD");
    if (globalMapPlanes.find(plane.getId()) == globalMapPlanes.end()) return;
//    cout<<"------------------------------ Before plane update: "<<endl;
//    globalMapPlanes.at(plane.getId()).print();

    globalMapPlanes.at(plane.getId()).updatePlaneParameters(plane);
//    globalMapPlanes.at(plane.getId()).print();
//    cout<<"------------------------------- After plane update: "<<endl;
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

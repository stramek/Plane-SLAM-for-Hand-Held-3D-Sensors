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
    long foundId = getIdByPlane(transformedPlane);

    ImageLoader imageLoader(500);
    imageLoader.getNextPair();
    ImagePair currentFrame = imageLoader.getNextPair(positionNumber + 1);

    if (foundId == -1) {
        assignIdToPlane(transformedPlane);
        planeUtils::visualizePlaneLocations(getGlobalMapVector(), transformedPlane, currentFrame.getRgb(), currentFrame.getRgb());
        waitKey();
        globalMapPlanes.insert(pair<long, Plane>(transformedPlane.getId(), transformedPlane));
        planeUtils::visualizePlaneLocations(getGlobalMapVector(), transformedPlane, currentFrame.getRgb(), currentFrame.getRgb());
        cout<<"New plane detected! Assigned id = "<<currentId<<endl;
        return tuple<long, bool, Plane>(currentId, true, transformedPlane);
    } else {
        vector<pair<Plane,Plane>> vec;
        Plane foundPlane = getPlaneById(foundId);
        vec.push_back(pair<Plane, Plane>(transformedPlane, foundPlane));
        planeUtils::visualizeSimilarPlanes(vec, currentFrame.getRgb(), currentFrame.getRgb());
        waitKey();

        cout<<"Plane already exists! id: "<<foundId<<endl;
        return tuple<long, bool, Plane>(foundId, false, Plane());
    }
}

Plane GlobalMap::getPlaneById(long id) {
    return getGlobalMapPlanes().at(id);
}

long GlobalMap::getIdByPlane(Plane &plane) {
    cout<<"***************************************"<<endl;
    for (auto &mapPlane : globalMapPlanes) {
        if (isAngleBetweenPlanesValid(plane, mapPlane.second)
            /*&& isDistanceBetweenPlanesValid(plane, mapPlane.second)*/
            && isHueDiffValid(plane, mapPlane.second)) {
            return mapPlane.second.getId();
        }
    }
    cout<<"***************************************"<<endl;
    return -1;
}

bool GlobalMap::isAngleBetweenPlanesValid(Plane &plane1, Plane &plane2) {
    bool isValid = plane1.getAngleBetweenTwoPlanes(plane2) < MAX_ANGLE_BETWEEN_PLANES_GLOBAL_MAP;
    if (isValid) {
        cout<<"Plane ANGLE is valid, value: " <<plane1.getAngleBetweenTwoPlanes(plane2)<<endl;
    } else {
        cout<<"Plane ANGLE is NOT valid, value: " <<plane1.getAngleBetweenTwoPlanes(plane2)<<endl;
    }
    return isValid;
}

bool GlobalMap::isDistanceBetweenPlanesValid(Plane &plane1, Plane &plane2) {
    bool isValid = planeUtils::getDistanceBetweenTwoPlanes(plane1, plane2) < MAX_DISTANCE_BETWEEN_PLANES_GLOBAL_MAP;
    if (isValid) {
        cout<<"Plane DISTANCE is valid, value: " <<planeUtils::getDistanceBetweenTwoPlanes(plane1, plane2)<<endl;
    } else {
        cout<<"Plane DISTANCE is NOT valid, value: " <<planeUtils::getDistanceBetweenTwoPlanes(plane1, plane2)<<endl;
    }
    return isValid;
}

bool GlobalMap::isHueDiffValid(Plane &plane1, Plane &plane2) {
    bool isValid = abs(plane1.getColor().getHue() - plane2.getColor().getHue()) < MAX_SIMILARITY_GLOBAL_MAP_VALUE;
    if (isValid) {
        cout<<"Plane COLOR is valid, value: " <<abs(plane1.getColor().getHue() - plane2.getColor().getHue())<<endl;
    } else {
        cout<<"Plane COLOR is NOT valid, value: " <<abs(plane1.getColor().getHue() - plane2.getColor().getHue())<<endl;
    }
    return isValid;
}

void GlobalMap::assignIdToPlane(Plane &plane) {
    plane.setId(++currentId);
}

void GlobalMap::updatePlane(Plane &plane) {
    if (plane.getId() == -1) throw runtime_error("MICHAU NIE PRZYDZIELIŁ ID XDDDDDDD");
    cout<<"------------------------------ Before plane update: ";
    globalMapPlanes.at(plane.getId()).print();
    globalMapPlanes.at(plane.getId()).updatePlaneParameters(plane);
    cout<<"------------------------------- After plane update: ";
    globalMapPlanes.at(plane.getId()).print();
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

//
// Created by mordimer on 12.03.17.
//

#include "include/algorithms/RansacPlaneDetector.h"

Plane RansacPlaneDetector::computePlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords ) {
    for (int i = 0; i < MAX_ITERATIONS_NUM; ++i) {
        Vector3d random3Points[3];
        getRandom3Points(pointsVector, random3Points);
        Plane plane(random3Points[0], random3Points[1], random3Points[2], imageCoords);
        int inlairesNumber = 0;
        for(auto point : pointsVector){
            if(plane.getDistanceFromPoint(point.position) < MAX_INLARIES_POINT_PLANE_DISTANCE)
                ++inlairesNumber;
        }
        if(inlairesNumber >= pointsVector.size()*INLAIERS_PERCENT_CONSENSUS){
            plane.insertPoints(pointsVector);
            plane.computeNormalVecDirection();

            return plane;
        }
    }
    return Plane();
}

void RansacPlaneDetector::getRandom3Points(const vector<Point3D> &pointsVector, Vector3d *random3Points) {
    random_device rd;
    mt19937 rng(rd());
    uniform_int_distribution<unsigned int> distribution(0, static_cast<unsigned int>(pointsVector.size() - 1));
    unsigned int randomPointsIndexes[3];
    for (int i = 0; i < 3; ++i) {
        while (1) {
            int counter = 0;
            randomPointsIndexes[i] = distribution(rng);
            for (int j = 0; j < i; ++j) {
                if(randomPointsIndexes[i] == randomPointsIndexes[j])
                    ++counter;
            }
            if (counter==0)
                break;
        }
        random3Points[i] = pointsVector.at(randomPointsIndexes[i]).position;
    }
}

Plane RansacPlaneDetector::getPlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords, const Mat *colorImage) {
    Plane plane = computePlane(pointsVector, imageCoords);
    if (plane.isValid()) {
        if (colorImage != nullptr) {
            plane.setColor(HSVColor(*colorImage));
        } else {
            plane.setColor(HSVColor(pointsVector));
        }
    }
    return plane;
}
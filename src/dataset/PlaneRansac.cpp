//
// Created by mordimer on 12.03.17.
//

#include "include/dataset/PlaneRansac.h"

Plane PlaneRansac::computePlane(const vector<Vector3f> &pointsVector) {
    for (int i = 0; i < MAX_ITERATIONS_NUM; ++i) {
        Vector3f random3Points[3];
        getRandom3Points(pointsVector, random3Points);
        Plane plane(random3Points[0], random3Points[1], random3Points[2]);
        int inlairesNumber = 0;
        for(Vector3f point : pointsVector){
            if(plane.getDistanceFromPoint(point) < MAX_INLARIES_POINT_PLANE_DISTANCE)
                ++inlairesNumber;
        }
        if(inlairesNumber >= pointsVector.size()*INLAIERS_PERCENT_CONSENSUS)
            return plane;
    }
    return Plane();
}

void PlaneRansac::getRandom3Points(const vector<Vector3f> &pointsVector, Vector3f *random3Points) {
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
        random3Points[i] = pointsVector.at(randomPointsIndexes[i]);
    }
}

Plane PlaneRansac::getPlane(const vector<Vector3f> &pointsVector) {
    return computePlane(pointsVector);
}

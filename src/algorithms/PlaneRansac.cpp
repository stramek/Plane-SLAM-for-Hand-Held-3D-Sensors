//
// Created by mordimer on 12.03.17.
//

#include "include/algorithms/PlaneRansac.h"

Plane PlaneRansac::computePlane(const vector<Vector3d> &pointsVector, const ImageCoords &imageCoords ) {
    for (int i = 0; i < MAX_ITERATIONS_NUM; ++i) {
        Vector3d random3Points[3];
        getRandom3Points(pointsVector, random3Points);
        Plane plane(random3Points[0], random3Points[1], random3Points[2], imageCoords);
        int inlairesNumber = 0;
        for(Vector3d point : pointsVector){
            if(plane.getDistanceFromPoint(point) < MAX_INLARIES_POINT_PLANE_DISTANCE)
                ++inlairesNumber;
        }
        if(inlairesNumber >= pointsVector.size()*INLAIERS_PERCENT_CONSENSUS)
            return plane;
    }
    return Plane();
}

void PlaneRansac::getRandom3Points(const vector<Vector3d> &pointsVector, Vector3d *random3Points) {
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

Plane PlaneRansac::getPlane(const vector<Vector3d> &pointsVector, const Mat &colorImage, const ImageCoords &imageCoords) {
    Plane plane = computePlane(pointsVector, imageCoords);
    if (plane.isValid()) plane.setColor(HSVColor(colorImage));
    return plane;
}

Plane PlaneRansac::getPlane(const vector<Vector3d> &pointsVector, const vector<Point3D> &points, const ImageCoords &imageCoords) {
    Plane plane = computePlane(pointsVector, imageCoords);
    if (plane.isValid()) plane.setColor(HSVColor(points));
    return plane;
}
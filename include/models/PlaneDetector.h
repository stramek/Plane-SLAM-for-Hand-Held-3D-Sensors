//
// Created by stramek on 28.05.17.
//

#ifndef PROJEKTMAGISTERSKI_PLANEDETECTOR_H
#define PROJEKTMAGISTERSKI_PLANEDETECTOR_H

#include "Plane.h"

class PlaneDetector {
public:
    virtual Plane getPlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords, const Mat *colorImage = nullptr, bool withAcceptedRange = false) = 0;
    Vector3d computeMean(const vector<Point3D> &pointsVector) {
        Vector3d mean(0, 0, 0);
        for (auto &vector : pointsVector) {
            mean += vector.position;
        }
        mean /= (double) pointsVector.size();
        return mean;
    }
};

#endif //PROJEKTMAGISTERSKI_PLANEDETECTOR_H
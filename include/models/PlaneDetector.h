//
// Created by stramek on 28.05.17.
//

#ifndef PROJEKTMAGISTERSKI_PLANEDETECTOR_H
#define PROJEKTMAGISTERSKI_PLANEDETECTOR_H

class PlaneDetector {
public:
    virtual Plane getPlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords, const Mat *colorImage = nullptr);
};

#endif //PROJEKTMAGISTERSKI_PLANEDETECTOR_H
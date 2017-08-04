//
// Created by mordimer on 26.02.17.
//

#include "include/algorithms/PcaPlaneDetector.h"

Vector3d PcaPlaneDetector::computeMean(const vector<Point3D> &pointsVector) {
    Vector3d mean(0, 0, 0);
    for (auto &vector : pointsVector) {
        mean += vector.position;
    }
    mean /= (double) pointsVector.size();
    return mean;
}

Mat33 PcaPlaneDetector::computeCovMatrix(const vector<Point3D> &pointsVector, const Vector3d &mean) {
    Mat33 cov(Mat33::Zero());

    for (auto &point : pointsVector) {
        cov += (point.position - mean) * (point.position - mean).transpose();
    }

    return cov;
}

Plane PcaPlaneDetector::computePlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords, bool withAcceptedRange) {
    if (pointsVector.size() < 3)
        return Plane();
    Vector3d mean = computeMean(pointsVector);
    Mat33 cov = computeCovMatrix(pointsVector, mean);
    EigenSolver<Mat33> eigenSolver(cov);

    int minIndex = 0;
    if (real(cov.eigenvalues()(1)) < real(cov.eigenvalues()(0))) {
        minIndex = 1;
        if (real(cov.eigenvalues()(2)) < real(cov.eigenvalues()(1)))
            minIndex = 2;
    } else if
            (real(cov.eigenvalues()(2)) < real(cov.eigenvalues()(1))) {
        minIndex = 2;
        if (real(cov.eigenvalues()(0)) < real(cov.eigenvalues()(2)))
            minIndex = 0;
    }

    if (withAcceptedRange || abs(cov.eigenvalues()(minIndex)) < PCA_MAX_ACCEPTED_DISTANCE) {
        auto eigenVectors = eigenSolver.eigenvectors();
        Vector3d normalVec = Vector3d(real(eigenVectors(0, minIndex)),
                                      real(eigenVectors(1, minIndex)), real(eigenVectors(2, minIndex)));

        Plane plane(normalVec, mean, pointsVector, imageCoords);
        plane.computeNormalVecDirection();

        return plane;
    }
    return Plane();
}



Plane PcaPlaneDetector::getPlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords, const Mat *colorImage, bool withAcceptedRange) {
    Plane plane = computePlane(pointsVector, imageCoords, withAcceptedRange);
    if (plane.isValid()) {
        if (colorImage != nullptr) {
            plane.setColor(HSVColor(*colorImage));
        } else {
            plane.setColor(HSVColor(pointsVector));
        }
    }
    return plane;
}

/*
Plane PcaPlaneDetector::getPlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords) {
    Plane plane = computePlane(pointsVector, imageCoords);
    if (plane.isValid()) plane.setColor(HSVColor(pointsVector));
    return plane;
}
*/

//
// Created by mordimer on 26.02.17.
//

#include "include/algorithms/PlanePca.h"

Vector3f PlanePca::computeMean(const vector<Vector3f> &pointsVector) {
    Vector3f mean(0, 0, 0);
    for (auto &vector : pointsVector) {
        mean += vector;
    }
    mean /= (double) pointsVector.size();
    return mean;
}

Mat33 PlanePca::computeCovMatrix(const vector<Vector3f> &pointsVector, const Vector3f &mean) {
    Mat33 cov(Mat33::Zero());

    for (auto &point : pointsVector) {
        cov += (point - mean) * (point - mean).transpose();
    }

    return cov;
}

Plane PlanePca::computePlane(const vector<Vector3f> &pointsVector, const Mat &colorImage, const ImageCoords &imageCoords,
                             bool reverseNormal) {
    Vector3f mean = computeMean(pointsVector);
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

    if (abs(cov.eigenvalues()(minIndex)) < PCA_MAX_ACCEPTED_DISTANCE) {
        auto eigenVectors = eigenSolver.eigenvectors();
        Vector3f normalVec = Vector3f(real(eigenVectors(0, minIndex)),
                                      real(eigenVectors(1, minIndex)), real(eigenVectors(2, minIndex)));

        Vector3f cameraAxisVec(0.0f, 0.0f, 1.0f);
        normalVec.normalize();
        float normalVecCameraAxisAngle = acosf(normalVec.dot(cameraAxisVec)) * 180.0f / M_PI;

        if(!(normalVecCameraAxisAngle > 85 && normalVecCameraAxisAngle < 95)){
            if (normalVecCameraAxisAngle > 90 && reverseNormal) {
                normalVec = -normalVec;
            }
        } else {
            if(reverseNormal){
                Vector3f cameraToPlaneVec = pointsVector.at((pointsVector.size() - 1) / 2);
                cameraToPlaneVec.normalize();
                float angle = acosf(normalVec.dot(cameraToPlaneVec)) * 180.0f / M_PI;
                if(angle < 90) normalVec = -normalVec;
            }
        }


        return Plane(normalVec, pointsVector.at(0), colorImage, pointsVector, imageCoords);
    }
    return Plane();
}

Plane PlanePca::getPlane(const vector<Vector3f> &pointsVector, const Mat &colorImage, const ImageCoords &imageCoords, bool reverseNormal) {
    return computePlane(pointsVector, colorImage, imageCoords, reverseNormal);
}

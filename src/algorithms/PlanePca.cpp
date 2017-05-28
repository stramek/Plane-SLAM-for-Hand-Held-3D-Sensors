//
// Created by mordimer on 26.02.17.
//

#include "include/algorithms/PlanePca.h"

Vector3d PlanePca::computeMean(const vector<Point3D> &pointsVector) {
    Vector3d mean(0, 0, 0);
    for (auto &vector : pointsVector) {
        mean += vector.position;
    }
    mean /= (double) pointsVector.size();
    return mean;
}

Mat33 PlanePca::computeCovMatrix(const vector<Point3D> &pointsVector, const Vector3d &mean) {
    Mat33 cov(Mat33::Zero());

    for (auto &point : pointsVector) {
        cov += (point.position - mean) * (point.position - mean).transpose();
    }

    return cov;
}

Plane PlanePca::computePlane(const vector<Point3D> &pointsVector, const ImageCoords &imageCoords) {
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

    if (abs(cov.eigenvalues()(minIndex)) < PCA_MAX_ACCEPTED_DISTANCE) {
        auto eigenVectors = eigenSolver.eigenvectors();
        Vector3d normalVec = Vector3d(real(eigenVectors(0, minIndex)),
                                      real(eigenVectors(1, minIndex)), real(eigenVectors(2, minIndex)));

        Vector3d cameraAxisVec(0.0f, 0.0f, 1.0f);
        normalVec.normalize();
        double normalVecCameraAxisAngle = acos(normalVec.dot(cameraAxisVec)) * 180.0 / M_PI;
        if(DEBUG){
            std::cout << "Vector: " << normalVec(0) << " " << normalVec(1) << " " << normalVec(2) << " angle: " << normalVecCameraAxisAngle << std::endl;
        }

        if(!(normalVecCameraAxisAngle > 85 && normalVecCameraAxisAngle < 95)){
            if (normalVecCameraAxisAngle > 90) {
                normalVec = -normalVec;
            }
        } else {
            Vector3d cameraToPlaneVec = pointsVector.at((pointsVector.size() - 1) / 2).position;
            cameraToPlaneVec.normalize();
            double angle = acos(normalVec.dot(cameraToPlaneVec)) * 180.0 / M_PI;
            if(angle < 90) normalVec = -normalVec;
        }


        return Plane(normalVec, pointsVector.at(0).position, pointsVector, imageCoords);
        //return Plane();
    }
    return Plane();
}

Plane PlanePca::getPlane(const vector<Point3D> &pointsVector, const Mat &colorImage, const ImageCoords &imageCoords) {
    Plane plane = computePlane(pointsVector, imageCoords);
    if (plane.isValid()) plane.setColor(HSVColor(colorImage));
    return plane;
}

Plane PlanePca::getPlane(const vector<Point3D> &pointsVector, const vector<Point3D> &points, const ImageCoords &imageCoords) {
    Plane plane = computePlane(pointsVector, imageCoords);
    if (plane.isValid()) plane.setColor(HSVColor(points));
    return plane;
}

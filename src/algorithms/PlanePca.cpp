//
// Created by mordimer on 26.02.17.
//

#include "include/algorithms/PlanePca.h"

void PlanePca::pointsVectorToMatrix(const vector<Vector3f> &pointsVector, MatrixXf &matrix) {
    matrix.resize(pointsVector.size(), 3);
    int i = 0;
    for (auto row : pointsVector) {
        matrix.row(i++) = row;
    }
}

MatrixXf PlanePca::computeCovMatrix(const MatrixXf &matrix) {
    MatrixXf centered = matrix.rowwise() - matrix.colwise().mean();
    return (centered.adjoint() * centered) / double(matrix.rows() - 1);
}

Vector3f PlanePca::computeMean(const vector<Vector3f> &pointsVector) {
    Vector3f mean(0, 0, 0);
    for (auto &vector : pointsVector) {
        mean += vector;
    }
    mean /= (double) pointsVector.size();
    return mean;
}

Mat33 PlanePca::computeCovMatrix2(const vector<Vector3f> &pointsVector, const Vector3f &mean) {
    Mat33 cov(Mat33::Zero());

    for (auto &point : pointsVector) {
        cov += (point - mean) * (point - mean).transpose();
    }

    return cov;
}

Plane PlanePca::computePlane(const vector<Vector3f> &pointsVector, const Mat &colorImage, const ImageCoords &imageCoords) {
    Vector3f mean = computeMean(pointsVector);
    Mat33 cov = computeCovMatrix2(pointsVector, mean);
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

    cout << "Value: " << to_string(abs(cov.eigenvalues()(minIndex))) << endl;
    if (abs(cov.eigenvalues()(minIndex)) < PCA_MAX_ACCEPTED_DISTANCE) {
        auto eigenVectors = eigenSolver.eigenvectors();
        Vector3f normalVec = Vector3f(real(eigenVectors(1, minIndex)),
                                      real(eigenVectors(0, minIndex)), real(eigenVectors(2, minIndex)));
        if (normalVec(2) < 0) normalVec = -normalVec;

        return Plane(normalVec, pointsVector.at(0), colorImage, pointsVector, imageCoords);
    }
    return Plane();
}

Plane PlanePca::getPlane(const vector<Vector3f> &pointsVector, const Mat &colorImage, const ImageCoords &imageCoords) {
    return computePlane(pointsVector, colorImage, imageCoords);
}

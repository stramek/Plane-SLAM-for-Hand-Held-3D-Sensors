//
// Created by mordimer on 26.02.17.
//

#include "include/dataset/PlanePca.h"

void PlanePca::vecContainerToMatrix(vector<Vector3f> pointsVector, MatrixXf &matrix) {
    matrix.resize(pointsVector.size(), 3);
    int i = 0;
    for (vector<Vector3f>::iterator it = pointsVector.begin(); it != pointsVector.end(); ++it) {
        matrix.row(i) = *it;
        ++i;
    }
}

MatrixXf PlanePca::computeCovMatrix(const MatrixXf &matrix) {
    MatrixXf centered = matrix.rowwise() - matrix.colwise().mean();
    return (centered.adjoint() * centered) / double(matrix.rows() - 1);
}

Plane PlanePca::computePlane(vector<Vector3f> pointsVector) {
    MatrixXf matrix;
    vecContainerToMatrix(pointsVector, matrix);
    MatrixXf covMatrix = computeCovMatrix(matrix);
    SelfAdjointEigenSolver<MatrixXf> eigenSolver;
    eigenSolver.compute(covMatrix);
    int minIndex;
    eigenSolver.eigenvalues().minCoeff(&minIndex);
    if (abs(eigenSolver.eigenvalues()(minIndex)) < 2) {
        Vector3f normalVec;
        MatrixXf eigenVectors = eigenSolver.eigenvectors();
        normalVec = eigenVectors.col(minIndex);
        return Plane(normalVec, pointsVector.at(0));
    }
    return Plane();
}

Plane PlanePca::getPlane(vector<Vector3f> pointsVector) {
    return computePlane(pointsVector);
}

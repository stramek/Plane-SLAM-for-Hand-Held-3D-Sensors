//
// Created by mordimer on 26.02.17.
//

#include "include/dataset/PlanePca.h"

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

Plane PlanePca::computePlane(const vector<Vector3f> &pointsVector) {
    MatrixXf matrix;
    pointsVectorToMatrix(pointsVector, matrix);
    MatrixXf covMatrix = computeCovMatrix(matrix);
    SelfAdjointEigenSolver<MatrixXf> eigenSolver;
    eigenSolver.compute(covMatrix);
    int minIndex;
    eigenSolver.eigenvalues().minCoeff(&minIndex);
    cout << "eigen values: " << endl << eigenSolver.eigenvalues() << endl;
    cout << "min index: " << minIndex << endl;
    cout << eigenSolver.eigenvalues()<< endl;
    if (abs(eigenSolver.eigenvalues()(minIndex)) < PCA_MAX_ACCEPTED_DISTANCE) {
        Vector3f normalVec;
        MatrixXf eigenVectors = eigenSolver.eigenvectors();
        normalVec = eigenVectors.col(minIndex);
        return Plane(normalVec, pointsVector.at(0));
    }
    return Plane();
}

Plane PlanePca::getPlane(const vector<Vector3f> &pointsVector) {
    return computePlane(pointsVector);
}

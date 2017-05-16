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

Plane PlanePca::computePlane(const vector<Vector3f> &pointsVector, const Mat& colorImage, const ImageCoords& imageCoords) {
    MatrixXf matrix;
    pointsVectorToMatrix(pointsVector, matrix);
    MatrixXf covMatrix = computeCovMatrix(matrix);
    SelfAdjointEigenSolver<MatrixXf> eigenSolver;
    eigenSolver.compute(covMatrix);
    int minIndex;
    std::cout << "qwe" << std::endl;
    eigenSolver.eigenvalues().minCoeff(&minIndex);
    if (abs(eigenSolver.eigenvalues()(minIndex)) < PCA_MAX_ACCEPTED_DISTANCE) {
        Vector3f normalVec;
        MatrixXf eigenVectors = eigenSolver.eigenvectors();
        normalVec = eigenVectors.col(minIndex);
        normalVec.normalize();
        Vector3f cameraAxis(0.0f, 0.0f, -1.0f);
        float anglePlaneVecCameraVec = acosf(normalVec.dot(cameraAxis))*180.0f/(float)M_PI;
        if(anglePlaneVecCameraVec > 90.0f){
            normalVec = -normalVec;
        }
        return Plane(normalVec, pointsVector.at(0), colorImage, pointsVector, imageCoords);
    }
    return Plane();
}

Plane PlanePca::getPlane(const vector<Vector3f> &pointsVector, const Mat& colorImage, const ImageCoords& imageCoords) {
    std::cout << "getPlane" << std::endl;
    return computePlane(pointsVector, colorImage, imageCoords);
}

//
// Created by mordimer on 26.02.17.
//

#include "include/algorithms/PlanePca.h"

typedef Matrix<float, 3, 3> Mat33;

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
    mean /= pointsVector.size();
}

MatrixXf PlanePca::computeCovMatrix2(const vector<Vector3f> &pointsVector, const Vector3f &mean) {
    cout << "ComputeCovMatix2" << endl;
    Mat33 cov(Mat33::Zero());

    for (auto &point : pointsVector) {
        Vector3f pMinMean = point - mean;
        cov += pMinMean * pMinMean.transpose();
    }

    return cov;
}

Plane
PlanePca::computePlane(const vector<Vector3f> &pointsVector, const Mat &colorImage, const ImageCoords &imageCoords) {
    Vector3f mean = computeMean(pointsVector);
    Mat33 cov = computeCovMatrix2(pointsVector, mean);
    EigenSolver<Mat33> eigenSolver(cov);
    //cov.eigenvalues().minCoeff(&minIndex);

    int minIndex = 0;
    if (std::real(cov.eigenvalues()(1)) < std::real(cov.eigenvalues()(0))) {
        minIndex = 1;
        if (std::real(cov.eigenvalues()(2)) < std::real(cov.eigenvalues()(1)))
            minIndex = 2;
    } else if
            (std::real(cov.eigenvalues()(2)) < std::real(cov.eigenvalues()(1))) {
        minIndex = 2;
        if (std::real(cov.eigenvalues()(0)) < std::real(cov.eigenvalues()(2)))
            minIndex = 0;
    }


    /*MatrixXf matrix;
    pointsVectorToMatrix(pointsVector, matrix);
    cout<<"Points inside matrix"<<matrix<<endl;
    MatrixXf covMatrix = computeCovMatrix(matrix);
    SelfAdjointEigenSolver<MatrixXf> eigenSolver;
    eigenSolver.compute(covMatrix);
    int minIndex;
    eigenSolver.eigenvalues().minCoeff(&minIndex);*/

    /*if (abs(cov.eigenvalues()(minIndex)) < PCA_MAX_ACCEPTED_DISTANCE) {
        Vector3f normalVec;
        Mat33 eigenVectors = eigenSolver.eigenvectors();
        normalVec = eigenVectors.col(minIndex);
        return Plane(normalVec, pointsVector.at(0), colorImage, pointsVector, imageCoords);
    }*/
    return Plane();
}

Plane PlanePca::getPlane(const vector<Vector3f> &pointsVector, const Mat &colorImage, const ImageCoords &imageCoords) {
    return computePlane(pointsVector, colorImage, imageCoords);
}

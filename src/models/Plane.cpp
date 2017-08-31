//
//  Plane.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 11.02.2017.
//

#include "include/models/Plane.h"

Plane::Plane() {}

Plane::Plane(const Vector3d &point1, const Vector3d &point2, const Vector3d &point3, const ImageCoords &imageCoords) {
    this->imageCoordsVec.push_back(imageCoords);
    computePlaneEquation(point1, point2, point3);
}

Plane::Plane(std::array<Vector3d, 3>, const Mat &colorImage) {
    color = HSVColor(colorImage);
}

Plane::Plane(Vector3d normalVec, Vector3d meanPoint, const vector<Point3D> &points, const ImageCoords &imageCoords) {
    planeNormalVec = normalVec;
    planeNormalVec.normalize();
    A = normalVec(0);
    B = normalVec(1);
    C = normalVec(2);
    D = A * meanPoint(0) + B * meanPoint(1) + C * meanPoint(2);
    this->points = points;
    this->imageCoordsVec.push_back(imageCoords);
    valid = true;
    this->meanPoint = meanPoint;
}

double Plane::getA() const {
    return A;
}

double Plane::getB() const {
    return B;
}

double Plane::getC() const {
    return C;
}

double Plane::getD() const {
    return D;
}

double Plane::getDistanceFromPoint(Vector3d point) const {
    return std::abs(A * point(0) + B * point(1) + C * point(2) - D)
           / sqrt(pow(A, 2.0) + pow(B, 2.0) + pow(C, 2.0));
}

void Plane::computePlaneEquation(const Vector3d &point1, const Vector3d &point2, const Vector3d &point3) {
    Eigen::Vector3d v = point1 - point2;
    Eigen::Vector3d w = point1 - point3;
    Eigen::Vector3d planeParameters = v.cross(w);
    planeParameters.normalize();
    A = planeParameters(0);
    B = planeParameters(1);
    C = planeParameters(2);
    D = A * point1(0) + B * point1(1) + C * point1(2);
    valid = true;
    planeNormalVec(0) = A;
    planeNormalVec(1) = B;
    planeNormalVec(2) = C;
}

bool Plane::isValid() const {
    return valid;
}

const HSVColor &Plane::getColor() const {
    return color;
}

Vector3d Plane::computePointOnPlaneFromTwoCoordinate(double firstCoordinate, double secondCoordinate) const {
    Vector3d pointToReturn;
    if (A == 0 && B == 0 && C == 0) {
        throw std::runtime_error("Plane equation error!");
    } else if (A == 0 && B == 0) {
        double z = D / B;
        pointToReturn(0) = firstCoordinate;
        pointToReturn(1) = secondCoordinate;
        pointToReturn(2) = z;
    } else if (A == 0 && C == 0) {
        double y = D / B;
        pointToReturn(0) = firstCoordinate;
        pointToReturn(1) = y;
        pointToReturn(2) = secondCoordinate;
    } else if (B == 0 && C == 0) {
        double x = D / A;
        pointToReturn(0) = x;
        pointToReturn(1) = firstCoordinate;
    } else if (C == 0) {
        double y = (D - A * firstCoordinate) / B;
        pointToReturn(0) = firstCoordinate;
        pointToReturn(1) = y;
        pointToReturn(2) = secondCoordinate;
    } else {
        double z = (D - A * firstCoordinate - B * secondCoordinate) / C;
        pointToReturn(0) = firstCoordinate;
        pointToReturn(1) = secondCoordinate;
        pointToReturn(2) = z;
    }
    return pointToReturn;
}

Vector3d Plane::getPlaneNormalVec() const {
    return planeNormalVec;
}

const ImageCoords &Plane::getImageCoords() const {
    //cout<<"Size: "<<imageCoordsVec.size()<<endl;
    return imageCoordsVec.at(0);
}

void Plane::setImageCoords(const ImageCoords &imageCoords) {
    Plane::imageCoordsVec.push_back(imageCoords);
}

const vector<Point3D> &Plane::getPoints() const {
    return points;
}

void Plane::setPoints(const vector<Point3D> &points) {
    Plane::points = points;
}

unsigned int Plane::getNumberOfPoints() const {
    return (unsigned int) points.size();
}

double Plane::getAngleBetweenTwoPlanes(const Plane &plane) const {
    Eigen::Vector3d planeNormalVec = plane.getPlaneNormalVec();

    double angleCos = (this->planeNormalVec(0) * plane.getPlaneNormalVec()(0) +
                        this->planeNormalVec(1) * plane.getPlaneNormalVec()(1) +
                        this->planeNormalVec(2) * plane.getPlaneNormalVec()(2)) /
                        this->planeNormalVec.norm() / planeNormalVec.norm();
    if (angleCos < -1) angleCos = -1.0;
    if (angleCos > 1) angleCos = 1.0;
    return acos(angleCos) * 180.0 / (double) M_PI;
}

void Plane::setColor(const HSVColor &color) {
    Plane::color = color;
}

void Plane::insertPoints(vector<Point3D> points) {
    this->points.insert(this->points.end(), points.begin(), points.end());
}

void Plane::insertImageCoords(vector<ImageCoords> imageCoordsVec) {
    this->imageCoordsVec.insert(this->imageCoordsVec.end(), imageCoordsVec.begin(), imageCoordsVec.end());
}

void Plane::mergePlane(Plane plane) {
    int colorSum = color.getHue() + plane.getColor().getHue();
    color.setHue((uint8_t) (colorSum / 2));
    insertPoints(plane.getPoints());
    insertImageCoords(plane.getImageCoordsVec());
}

const vector<ImageCoords> &Plane::getImageCoordsVec() const {
    return imageCoordsVec;
}

Point3D Plane::getCentralPoint() const {
    return points[(points.size() - 1) / 2];
}

void Plane::computeNormalVecDirection(){
    Vector3d cameraAxisVec(0.0f, 0.0f, 1.0f);
    planeNormalVec.normalize();
    double normalVecCameraAxisAngle = acos(planeNormalVec.dot(cameraAxisVec)) * 180.0 / M_PI;

    if(!(normalVecCameraAxisAngle > 85 && normalVecCameraAxisAngle < 95)){
        if (normalVecCameraAxisAngle > 90) {
            planeNormalVec = -planeNormalVec;
            A = -A;
            B = -B;
            C = -C;
            D = -D;
        }
    } else {
        if(points.size() != 0){
            Vector3d cameraToPlaneVec = points.at((points.size() - 1) / 2).position;
            cameraToPlaneVec.normalize();
            double angle = acos(planeNormalVec.dot(cameraToPlaneVec)) * 180.0 / M_PI;
            if(angle < 90) {
                planeNormalVec = -planeNormalVec;
                A = -A;
                B = -B;
                C = -C;
                D = -D;
            }
        }
    }

}

Plane::Plane(double D, const Vector3d &planeNormalVec) : D(D), planeNormalVec(planeNormalVec) {
    A = planeNormalVec(0);
    B = planeNormalVec(1);
    C = planeNormalVec(2);
}

long Plane::getId() {
    return id;
}

void Plane::setId(long id) {
    Plane::id = id;
}

void Plane::print() {
    std::cout << "A: " << A << " B: " << B << " C: " << C << " D: " << D << std::endl;
}

const Vector3d &Plane::getMeanPoint() const {
    return meanPoint;
}

Plane Plane::getPlaneSeenFromGlobalCamera(PosOrient &posOrient) {
    Quaterniond q = posOrient.getQuaternion().conjugate();
    auto rotMatrix = q.toRotationMatrix();
    Matrix4d matrix4d;
    Vector3d meanPoint = getMeanPoint();
    Vector4d meanPoint4;
    matrix4d.setZero();

    matrix4d.topLeftCorner(3, 3) = rotMatrix;
    matrix4d.topRightCorner(3, 1) = -posOrient.getPosition();
    matrix4d(3, 3) = 1;
    meanPoint4.topRightCorner(3, 1) = meanPoint;
    meanPoint4(3) = 1;
    Vector3d norm = rotMatrix * getPlaneNormalVec();
    Vector4d newPosition = matrix4d * meanPoint4;

    Plane plane = *this;
    plane.A = norm(0);
    plane.B = norm(1);
    plane.C = norm(2);
    plane.D = A * newPosition(0) + B * newPosition(1) + C * newPosition(2);
    plane.planeNormalVec(0) = plane.A;
    plane.planeNormalVec(1) = plane.B;
    plane.planeNormalVec(2) = plane.C;

    plane.transformPointsToGlobal(posOrient);

    return plane;
}

Vector4d Plane::getPlaneParameterInLocalPos(PosOrient &posOrient) {
    Quaterniond q = posOrient.getQuaternion();
    auto rotMatrix = q.toRotationMatrix();
    Matrix4d matrix4d;
    Vector3d meanPoint = getMeanPoint();
    Vector4d meanPoint4;
    matrix4d.setZero();

    matrix4d.topLeftCorner(3, 3) = rotMatrix;
    matrix4d.topRightCorner(3, 1) = posOrient.getPosition();
    matrix4d(3, 3) = 1;
    meanPoint4.topRightCorner(3, 1) = meanPoint;
    meanPoint4(3) = 1;
    Vector3d norm = rotMatrix * getPlaneNormalVec();
    Vector4d newPosition = matrix4d * meanPoint4;

    Vector4d toReturn;

    toReturn(0) = norm(0);
    toReturn(1)= norm(1);
    toReturn(2) = norm(2);
    toReturn(3) = A * newPosition(0) + B * newPosition(1) + C * newPosition(2);


    return toReturn;
}


void Plane::updatePlaneParameters(Plane &plane) {
    A = plane.getA();
    B = plane.getB();
    C = plane.getC();
    D = plane.getD();
    planeNormalVec = plane.getPlaneNormalVec();
}


void Plane::transformPointsToGlobal(PosOrient &posOrient) {
    Quaterniond q = posOrient.getQuaternion().conjugate();
    auto rotMatrix = q.toRotationMatrix();
    auto translation = -posOrient.getPosition();
    for (auto &point : points) {
        point.position = rotMatrix * point.position + translation;
    }
}

bool Plane::isWasMatched() const {
    return wasMatched;
}

void Plane::setWasMatched(bool wasMatched) {
    Plane::wasMatched = wasMatched;
}

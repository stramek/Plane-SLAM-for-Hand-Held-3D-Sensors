//
//  Plane.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 11.02.2017.
//
//

#include "include/models/Plane.h"

Plane::Plane() {}

Plane::Plane(Vector3d point1, Vector3d point2, Vector3d point3, const ImageCoords &imageCoords) {
    this->imageCoordsVec.push_back(imageCoords);
}

Plane::Plane(std::array<Vector3d, 3>, const Mat& colorImage) {
    color = HSVColor(colorImage);
}

Plane::Plane(Vector3d normalVec, Vector3d point, const vector<Point3D> &points, const ImageCoords &imageCoords) {
    planeNormalVec = normalVec;
    planeNormalVec.normalize();
    A = normalVec(0);
    B = normalVec(1);
    C = normalVec(2);
    D = A * point(0) + B * point(1) + C * point(2);
    this->points = points;
    this->imageCoordsVec.push_back(imageCoords);
    valid = true;
}

Plane::Plane(Vector3d normalVec, double D, vector<Point3D> points, vector<ImageCoords> imageCoordsVec, HSVColor color){
    this->planeNormalVec = normalVec;
    this->D = D;
    this->A = normalVec(0);
    this->B = normalVec(1);
    this->C = normalVec(2);
    this->points = points;
    this->imageCoordsVec = imageCoordsVec;
    this->color = color;
    valid = true;
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

double Plane::getDistanceFromPoint(Vector3d point) {
    return std::abs(A * point(0) + B * point(1) + C * point(2) + D)
           / sqrtf(powf(A, 2) + powf(B, 2) + powf(C, 2));
}

void Plane::computePlaneEquation(Vector3d point1, Vector3d point2, Vector3d point3) {
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
    if(A == 0 && B == 0 && C == 0){
        throw std::runtime_error("Plane equation error!");
    } else if( A == 0 && B == 0){
        double z = D / B;
        pointToReturn(0) = firstCoordinate;
        pointToReturn(1) = secondCoordinate;
        pointToReturn(2) = z;
    } else if(A == 0 && C == 0){
        double y = D / B;
        pointToReturn(0) = firstCoordinate;
        pointToReturn(1) = y;
        pointToReturn(2) = secondCoordinate;
    } else if(B==0 && C==0){
        double x = D / A;
        pointToReturn(0) = x;
        pointToReturn(1) = firstCoordinate;
    } else if(C == 0){
        double y = (D - A * firstCoordinate) / B;
        pointToReturn(0) = firstCoordinate;
        pointToReturn(1) = y;
        pointToReturn(2) = secondCoordinate;
    } else{
        double z = (D - A * firstCoordinate - B * secondCoordinate) / C;
        pointToReturn(0) = firstCoordinate;
        pointToReturn(1) = secondCoordinate;
        pointToReturn(2) = z;
    }
    return  pointToReturn;
}

Vector3d Plane::getPlaneNormalVec() const{
    return planeNormalVec;
}

const ImageCoords &Plane::getImageCoords() const {
    return imageCoordsVec[0];
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
    return (unsigned int)points.size();
}

double Plane::getAngleBetweenTwoPlanes(const Plane &plane) const {
    Eigen::Vector3d planeNormalVec = plane.getPlaneNormalVec();

    double angleCos = this->planeNormalVec.dot(planeNormalVec) / this->planeNormalVec.norm() / planeNormalVec.norm();
    if(angleCos < -1) angleCos = -1.0;
    if(angleCos > 1) angleCos = 1.0;
    double angle = acos(angleCos)*180.0/(double)M_PI;

    return angle;
}

void Plane::setColor(const HSVColor &color) {
    Plane::color = color;
}

void Plane::insertPoints(vector<Point3D> points){
    this->points.insert(this->points.end(), points.begin(), points.end());
}

void Plane::insertImageCoords(vector<ImageCoords> imageCoordsVec){
    this->imageCoordsVec.insert(this->imageCoordsVec.end(), imageCoordsVec.begin(), imageCoordsVec.end());
}

void Plane::mergePlane(Plane plane){
    int colorSum = color.getHue() + plane.getColor().getHue();
    color.setHue((uint8_t)(colorSum / 2));
    insertPoints(plane.getPoints());
    insertImageCoords(plane.getImageCoordsVec());
}

const vector<ImageCoords> &Plane::getImageCoordsVec() const {
    return imageCoordsVec;
}

Point3D Plane::getCentralPoint() const {
    return points[(points.size() - 1) / 2];
}


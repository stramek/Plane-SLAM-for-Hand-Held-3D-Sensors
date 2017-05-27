//
// Created by mordimer on 15.05.17.
//

#include <include/utils/constants.h>
#include "include/models/PointCloud.h"

PointCloud::PointCloud() {
    PHCPModel = PHCP_MODEL;
}

void PointCloud::depth2cloud(cv::Mat &depthImage, cv::Mat RGB, unsigned int imgStartX, unsigned int imgStartY){
    Eigen::Vector3d point;
    points3D.clear();


    for (unsigned int i = 0; i < depthImage.rows; i++) {
        for (unsigned int j = 0; j < depthImage.cols; j++) {
            double depthM = double(depthImage.at<uint16_t>(i, j)) / 5000.0f;
            getPoint(j + imgStartX, i + imgStartY, depthM, point);
            Point3D pointPCL;
            pointPCL.x = point(0);
            pointPCL.y = -point(1);
            pointPCL.z = -point(2);

            pointPCL.red = RGB.at<cv::Vec<uchar, 3>>(i, j).val[2];
            pointPCL.green = RGB.at<cv::Vec<uchar, 3>>(i, j).val[1];
            pointPCL.blue = RGB.at<cv::Vec<uchar, 3>>(i, j).val[0];

            points3D.push_back(pointPCL);

            points.push_back(Eigen::Vector3d(pointPCL.x, pointPCL.y, pointPCL.z));
        }
    }
}

void PointCloud::getPoint(unsigned int u, unsigned int v, double depth, Eigen::Vector3d &point3D) {
    Eigen::Vector3d point(u, v, 1);
    point3D = depth * PHCPModel * point;
}

void PointCloud::setPHCPModel(Eigen::Matrix<double, 3, 3> model) {
    this->PHCPModel = model;
}

void PointCloud::clear(){
    points3D.clear();
    points.clear();
}

void PointCloud::push_back(Point3D point3D){
    points3D.push_back(point3D);
    Vector3d point(point3D.x, point3D.y, point3D.z);
    points.push_back(point);
}

const std::vector<Point3D> &PointCloud::getPoints3D() const {
    return points3D;
}

const std::vector<Eigen::Vector3d> &PointCloud::getPoints() const {
    return points;
}
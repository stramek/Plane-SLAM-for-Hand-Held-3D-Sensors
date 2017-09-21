//
// Created by mordimer on 15.05.17.
//

#ifndef PROJEKTMAGISTERSKI_POINTCLOUD_H
#define PROJEKTMAGISTERSKI_POINTCLOUD_H


#include "Point3D.h"
#include <iostream>
#include <vector>
#include <opencv2/core/mat.hpp>
#include <Eigen/Dense>
#include <include/models/PosOrient.h>

class PointCloud {
private:
    std::vector<Point3D> points3D;
    Eigen::Matrix<double, 3, 3> PHCPModel;

    void getPoint(unsigned int u, unsigned int v, double depth, Eigen::Vector3d &point3D);

public:
    PointCloud();
    void depth2cloud(cv::Mat &depthImage, cv::Mat RGB, unsigned int imgStartX = 0, unsigned int imgStartY = 0);
    void setPHCPModel(Eigen::Matrix<double, 3, 3> model);
    void push_back(Point3D point3D);
    void clear();
    const std::vector<Point3D> &getPoints3D() const;

    void depth2cloud(cv::Mat &depthImage, cv::Mat &RGB, const PosOrient &posOrient);
};


#endif //PROJEKTMAGISTERSKI_POINTCLOUD_H

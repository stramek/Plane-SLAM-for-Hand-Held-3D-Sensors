/** @file QVisualizer.h
 *
 * implementation - QGLVisualizer
 *
 */

#ifndef QVISUALIZER_H_INCLUDED
#define QVISUALIZER_H_INCLUDED

#include <Eigen/Dense>
#include <QGLViewer/qglviewer.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <QKeyEvent>
#include <GL/glut.h>
#include <libfreenect2/registration.h>
#include "include/models/Point3D.h"

using namespace libfreenect2;

/// Homogeneous representation of SE(3) rigid body transformations
typedef Eigen::Transform<double, 3, Eigen::Affine> Mat34;

/// Map implementation
class QGLVisualizer : public QGLViewer {
public:
    /// Construction
    QGLVisualizer(void);

    /// Updates cloud
    void updateCloud(cv::Mat RGB, cv::Mat D);

    void updateCloud(Registration *registration, Frame *undistorted, Frame *registered);

    void getPoint(unsigned int u, unsigned int v, float depth, Eigen::Vector3d &point3D);

    void depth2cloud(cv::Mat &depthImage, cv::Mat RGB);

    void setPHCPModel(Eigen::Matrix<double, 3, 3> model);

    const std::vector<Point3D> &getPointCloud() const;

public:

    void keyPressEvent(QKeyEvent *event);

    /// Destruction
    ~QGLVisualizer(void);

private:
    Eigen::Matrix<double, 3, 3> PHCPModel;

    /// draw objects
    void draw();

    /// draw objects
    void animate();

    /// initialize visualizer
    void init();


private:

    bool shadowFlag = false;

    /// object pose
    Mat34 objectPose;

    /// object pose
    Mat34 cameraPose;

    std::mutex mtxCamera;

    std::vector<Point3D> pointCloud;
};

#endif // QVISUALIZER_H_INCLUDED

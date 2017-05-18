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
#include <cstdint>
#include <stdint-gcc.h>
#include "include/kinect/main.h"
#include "include/models/PointCloud.h"
#include "include/models/Plane.h"

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

    void updatePlanes(std::vector<Plane> &planes);

    void updateCloud(Registration *registration, Frame *undistorted, Frame *registered);

    void getPoint(unsigned int u, unsigned int v, double depth, Eigen::Vector3d &point3D);

    void depth2cloud(cv::Mat &depthImage, cv::Mat RGB);

    void setPHCPModel(Eigen::Matrix<double, 3, 3> model);

    const PointCloud &getPointCloud() const;

    bool isProgramFinished() const;

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

public:
    void setProgramFinished(bool programFinished);

private:

    bool programFinished = false;

    /// object pose
    Mat34 objectPose;

    /// object pose
    Mat34 cameraPose;

    std::mutex mtxCamera;

    PointCloud pointCloud;

    std::vector<Plane> planes;
};

#endif // QVISUALIZER_H_INCLUDED

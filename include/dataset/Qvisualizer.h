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
#include <GL/glut.h>

/// Homogeneous representation of SE(3) rigid body transformations
typedef Eigen::Transform<double, 3, Eigen::Affine> Mat34;

/// Map implementation
class QGLVisualizer : public QGLViewer {
public:

    struct Point3D {
        double x;
        double y;
        double z;
        unsigned char red;
        unsigned char green;
        unsigned char blue;
    };

    /// Construction
    QGLVisualizer(void);

    /// Updates cloud
    void updateCloud(cv::Mat RGB, cv::Mat D);

    void addToCloud(std::vector<Eigen::Vector3f> TestPoints);

    void getPoint(unsigned int u, unsigned int v, float depth, Eigen::Vector3d &point3D);

    void depth2cloud(cv::Mat &depthImage, cv::Mat RGB);

    void setPGCPModel(Eigen::Matrix<double, 3, 3> PHCPModel);

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

    bool shadowFlag = false;

    /// object pose
    Mat34 objectPose;

    /// object pose
    Mat34 cameraPose;

    std::mutex mtxCamera;

    std::vector<Point3D> PointCloud;

    std::vector<Eigen::Vector3f> TestPoints;
};

#endif // QVISUALIZER_H_INCLUDED

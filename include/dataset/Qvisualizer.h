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

    /// Construction
    QGLVisualizer(void);
    
    /// Updates cloud
    void updateCloud(cv::Mat RGB, cv::Mat D);

    void getPoint(unsigned int u, unsigned int v, float depth, Eigen::Vector3d &point3D);

    void depth2cloud(cv::Mat &depthImage, cv::Mat RGB);

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

    /// generate help string
    std::string help() const;

    bool shadowFlag = false;

    /// object pose
    Mat34 objectPose;

    /// object pose
    Mat34 cameraPose;

    std::mutex mtxCamera;

    struct Point3D {
        float x;
        float y;
        float z;
        unsigned char red;
        unsigned char green;
        unsigned char blue;
    };

    std::vector<Point3D> PointCloud;
};

#endif // QVISUALIZER_H_INCLUDED

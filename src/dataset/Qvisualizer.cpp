#include <cstdint>
#include <stdint-gcc.h>
#include "../../include/dataset/Qvisualizer.h"

QGLVisualizer::QGLVisualizer(void) : objectPose(Mat34::Identity()), cameraPose(Mat34::Identity()) {
    objectPose(0, 3) = 2.0;
}

/// Destruction
QGLVisualizer::~QGLVisualizer(void) {}

/// draw objects
void QGLVisualizer::draw() {
    double GLmat[16] = {objectPose(0, 0), objectPose(1, 0), objectPose(2, 0), 0, objectPose(0, 1), objectPose(1, 1),
                        objectPose(2, 1), 0, objectPose(0, 2), objectPose(1, 2), objectPose(2, 2), 0, objectPose(0, 3),
                        objectPose(1, 3), objectPose(2, 3), 1};
    glPushMatrix();
    glMultMatrixd(GLmat);
    glutSolidTeapot(0.1);
    glPopMatrix();

    glPushMatrix();
    glBegin(GL_POINTS);
    for (Point3D i : PointCloud) {
        glColor3f(i.red / 255.0f, i.green / 255.0f, i.blue / 255.0f);
        glVertex3f(i.x, i.y, i.z);
    }
    glEnd();
    glPopMatrix();

}

/// draw objects
void QGLVisualizer::animate() {
}

/// initialize visualizer
void QGLVisualizer::init() {
    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);

    camera()->setZNearCoefficient(0.00001f);
    camera()->setZClippingCoefficient(100.0);

    setBackgroundColor(QColor(255, 155, 155));

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    startAnimation();
}

/// Updates cloud
void QGLVisualizer::updateCloud(cv::Mat RGB, cv::Mat D) {
    depth2cloud(D, RGB);
}

void QGLVisualizer::getPoint(unsigned int u, unsigned int v, float depth,
                             Eigen::Vector3d &point3D) {
    Eigen::Vector3d point(u, v, 1);
    point3D = depth * PHCPModel * point;
}

/// Convert disparity image to point cloud
void QGLVisualizer::depth2cloud(cv::Mat &depthImage, cv::Mat RGB) {
    Eigen::Vector3d point;
    PointCloud.clear();
    for (unsigned int i = 0; i < depthImage.rows; i++) {
        for (unsigned int j = 0; j < depthImage.cols; j++) {
            if (depthImage.at<uint16_t>(i, j) > 800 && depthImage.at<uint16_t>(i, j) < 8500) {
                float depthM = float(depthImage.at<uint16_t>(i, j)) * 0.001;
                getPoint(j, i, depthM, point);
                Point3D pointPCL;
                pointPCL.x = point(0);
                pointPCL.y = point(1);
                pointPCL.z = point(2);

                pointPCL.red = RGB.at<cv::Vec<uchar, 3>>(i, j).val[2];
                pointPCL.green = RGB.at<cv::Vec<uchar, 3>>(i, j).val[1];
                pointPCL.blue = RGB.at<cv::Vec<uchar, 3>>(i, j).val[0];
                PointCloud.push_back(pointPCL);
            }

        }
    }
}
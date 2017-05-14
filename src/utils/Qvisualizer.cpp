#include "include/utils/Qvisualizer.h"

QGLVisualizer::QGLVisualizer(void) : objectPose(Mat34::Identity()), cameraPose(Mat34::Identity()) {
    objectPose(0, 3) = 2.0;
}

/// Destruction
QGLVisualizer::~QGLVisualizer(void) {}

//}

/// draw objects
void QGLVisualizer::draw() {
    glPushMatrix();
    glBegin(GL_POINTS);
    for (Point3D i : pointCloud) {
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

    setBackgroundColor(QColor(100, 100, 100));

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

void QGLVisualizer::getPoint(unsigned int u, unsigned int v, float depth, Eigen::Vector3d &point3D) {
    Eigen::Vector3d point(u, v, 1);
    point3D = depth * PHCPModel * point;
}

void QGLVisualizer::setPHCPModel(Eigen::Matrix<double, 3, 3> model) {
    this->PHCPModel = model;
}

/// Convert disparity image to point cloud
void QGLVisualizer::depth2cloud(cv::Mat &depthImage, cv::Mat RGB) {
    Eigen::Vector3d point;
    pointCloud.clear();

    float maxX = -100, maxY = -100, maxZ = -100;
    float minX = 1000, minY = 1000, minZ = 1000;

    for (unsigned int i = 0; i < depthImage.rows; i++) {
        for (unsigned int j = 0; j < depthImage.cols; j++) {
            float depthM = float(depthImage.at<uint16_t>(i, j)) / 5000.0f;
            getPoint(j, i, depthM, point);
            Point3D pointPCL;
            pointPCL.x = point(0);
            pointPCL.y = -point(1);
            pointPCL.z = -point(2);
            if(maxX < pointPCL.x) maxX = pointPCL.x;
            if(maxY < pointPCL.y) maxY = pointPCL.y;
            if(maxZ < pointPCL.z) maxZ = pointPCL.z;

            if(minX > pointPCL.x) minX = pointPCL.x;
            if(minY > pointPCL.y) minY = pointPCL.y;
            if(minZ > pointPCL.z) minZ = pointPCL.z;


            pointPCL.red = RGB.at<cv::Vec<uchar, 3>>(i, j).val[2];
            pointPCL.green = RGB.at<cv::Vec<uchar, 3>>(i, j).val[1];
            pointPCL.blue = RGB.at<cv::Vec<uchar, 3>>(i, j).val[0];

            pointCloud.push_back(pointPCL);
        }
    }

    std::cout<< "minX " << minX << " minY " << minY << " minZ " << minZ << std::endl;
    std::cout<< "maxX " << maxX << " maxY " << maxY << " maxZ " << maxZ << std::endl;
}

void QGLVisualizer::updateCloud(Registration *registration, Frame *undistorted,
                                Frame *registered) {
    pointCloud.clear();

    for (int r = 0; r < undistorted->height; ++r) {
        for (int c = 0; c < undistorted->width; ++c) {
            float x, y, z, color;
            registration->getPointXYZRGB(undistorted, registered, r, c, x, y, z, color);
            const uint8_t *p = reinterpret_cast<uint8_t *>(&color);
            if (!isnanf(z)) {
                Point3D point3D(-x, -y, -z, p[2], p[1], p[0]);
                pointCloud.push_back(point3D);
            }
        }
    }
}

void QGLVisualizer::keyPressEvent(QKeyEvent *event) {
    int key = event->key();

    switch (key) {
        case Qt::Key_Q:
            setProgramFinished(true);
            close();
            break;
        default:
            QGLViewer::keyPressEvent(event);
    }
}

const std::vector<Point3D> &QGLVisualizer::getPointCloud() const {
    return pointCloud;
}

bool QGLVisualizer::isProgramFinished() const {
    return programFinished;
}

void QGLVisualizer::setProgramFinished(bool programFinished) {
    QGLVisualizer::programFinished = programFinished;
}

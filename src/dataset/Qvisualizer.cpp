#include "../include/Qvisualizer.h"
#include <memory>
#include <cmath>
#include <stdexcept>
#include <chrono>
#include <GL/glut.h>

/// A single instance of Visualizer
QGLVisualizer::Ptr visualizer;

QGLVisualizer::QGLVisualizer(void) : objectPose(Mat34::Identity()), cameraPose(Mat34::Identity()){
    objectPose(0,3)=2.0;
}

/// Destruction
QGLVisualizer::~QGLVisualizer(void) {
    //#ifdef DEBUG
    //std::cout << "QGLVisualizer destructor\n";
    //#endif
}

/// draw objects
void QGLVisualizer::draw(){
    double GLmat[16]={objectPose(0,0), objectPose(1,0), objectPose(2,0), 0, objectPose(0,1), objectPose(1,1), objectPose(2,1), 0, objectPose(0,2), objectPose(1,2), objectPose(2,2), 0, objectPose(0,3), objectPose(1,3), objectPose(2,3), 1};
    glPushMatrix();
    glMultMatrixd(GLmat);
    glutSolidTeapot(0.1);
    glPopMatrix();

    glPushMatrix();
    glBegin(GL_POINTS);
    for(Point3D i : PointCloud)
    {
        glColor3f(i.red/255.0f, i.green/255.0f, i.blue/255.0f);
        glVertex3f(i.x, i.y, i.z);
    }
    glEnd();
    glPopMatrix();

}

/// draw objects
void QGLVisualizer::animate(){
}



/// initialize visualizer
void QGLVisualizer::init(){
    // Light setup
//    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0);
//    GLfloat specular_color[4] = { 0.99f, 0.99f, 0.99f, 1.0f };
//    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  specular_color);

//    //Set global ambient light
//    GLfloat black[] = {0.99f, 0.99f, 0.99f, 1.0f};
//    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, black);

    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
    // Restore previous viewer state.
    //restoreStateFromFile();

    camera()->setZNearCoefficient(0.00001f);
    camera()->setZClippingCoefficient(100.0);

    setBackgroundColor(QColor(255, 155, 155));

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

     // Opens help window
    help();

    startAnimation();
}

/// Updates cloud
void QGLVisualizer::updateCloud(cv::Mat RGB, cv::Mat D){
    depth2cloud(D, RGB);
}

/// generate help string
std::string QGLVisualizer::help() const{
    std::string text("S i m p l e V i e w e r");
    text += "Use the mouse to move the camera around the object. ";
    text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
    text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
    text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
    text += "Simply press the function key again to restore it. Several keyFrames define a ";
    text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
    text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
    text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
    text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
    text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
    text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
    text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. ";
    text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
    text += "Press <b>Escape</b> to exit the viewer.";
    return text;
}

void QGLVisualizer::getPoint(unsigned int u, unsigned int v, float depth,
Eigen::Vector3d& point3D){
    Eigen::Vector3d point(u, v, 1);
    point3D = depth*PHCPModel*point;
}

/// Convert disparity image to point cloud
void QGLVisualizer::depth2cloud(cv::Mat& depthImage, cv::Mat RGB){
    Eigen::Vector3d point;
    PointCloud.clear();
    for (unsigned int i=0;i<depthImage.rows;i++){
        for (unsigned int j=0;j<depthImage.cols;j++){
            if(depthImage.at<uint16_t>(i,j)>800&&depthImage.at<uint16_t>(i,j)<8500){
                float depthM = float(depthImage.at<uint16_t>(i,j))*0.001;
                getPoint(j,i,depthM,point);
                Point3D pointPCL;
                pointPCL.x = point(0);
                pointPCL.y = point(1);
                pointPCL.z = point(2);

                pointPCL.red = RGB.at<cv::Vec<uchar, 3>>(i, j).val[2];
                pointPCL.green = RGB.at<cv::Vec<uchar, 3>>(i, j).val[1];
                pointPCL.blue= RGB.at<cv::Vec<uchar, 3>>(i, j).val[0];
                PointCloud.push_back(pointPCL);
            }

        }
    }
}


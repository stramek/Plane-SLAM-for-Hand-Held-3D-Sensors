//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "../../include/kinect/main.h"

bool protonect_shutdown = false;

Freenect2 freenect2;
Freenect2Device *dev = nullptr;
PacketPipeline *pipeline = nullptr;

//void sigint_handler(int s) {
//    protonect_shutdown = true;
//}

void quitIfDeviceNotConnected() {
    if (freenect2.enumerateDevices() == 0) {
        throw std::runtime_error("Didn't find any kinect.");
    }
}

void setDepthProcessor() {
    if (!pipeline) {
        pipeline = new OpenGLPacketPipeline();
    }
}

void openDevice() {
    string serial = freenect2.getDefaultDeviceSerialNumber();
    dev = pipeline ? freenect2.openDevice(serial, pipeline) : freenect2.openDevice(serial);

    if (dev == 0) {
        throw std::runtime_error("Failure opening device!");
    }
}


int main(int argc, char **argv) {
    QApplication application(argc, argv);
    glutInit(&argc, argv);
    QGLVisualizer visualizer;
    visualizer.setWindowTitle("Dataset viewer");
    //visualizer.setPHCPModel(PHCP_MODEL);
    visualizer.show();

    quitIfDeviceNotConnected();
    setDepthProcessor();
    openDevice();

    SyncMultiFrameListener listener(Frame::Color | Frame::Depth | Frame::Ir);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    FrameMap frames;
    Registration *registration = new Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

    Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;

    while (!protonect_shutdown) {

        listener.waitForNewFrame(frames);
        Frame *rgb = frames[Frame::Color];
        Frame *ir = frames[Frame::Ir];
        Frame *depth = frames[Frame::Depth];

        //Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        //Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
        //Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

        //resize(rgbmat, rgbmat, Size(), 0.3, 0.3, INTER_LINEAR);

        //imshow("rgb", rgbmat);
        //imshow("ir", irmat / 4500.0f);
        //imshow("depth", depthmat / 4500.0f);

        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

        //Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
        //Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
        //Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

        //resize(rgbd2, rgbd2, Size(), 0.3, 0.3, INTER_LINEAR);

        //imshow("undistorted", depthmatUndistorted / 4500.0f);
        //imshow("registered", rgbd);
        //imshow("depth2RGB", rgbd2 / 4500.0f);



        //visualizer.updateCloud(registration, &undistorted, &registered);


        int key = waitKey(1000);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        listener.release(frames);
    }

    dev->stop();
    dev->close();

    delete registration;
    return application.exec();
}

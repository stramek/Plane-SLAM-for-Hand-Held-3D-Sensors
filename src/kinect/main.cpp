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
        pipeline = new OpenCLPacketPipeline();
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
    visualizer.setWindowTitle("Kinect pointcloud");
    visualizer.show();

    quitIfDeviceNotConnected();
    setDepthProcessor();
    openDevice();

    SyncMultiFrameListener listener(Frame::Color | Frame::Depth);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    FrameMap frames;
    Registration *registration = new Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    Frame undistorted(512, 424, 4), registered(512, 424, 4);

    Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;

    while (!protonect_shutdown) {
        listener.waitForNewFrame(frames);
        Frame *rgb = frames[Frame::Color];
        Frame *depth = frames[Frame::Depth];

        registration->apply(rgb, depth, &undistorted, &registered, true);
        visualizer.updateCloud(registration, &undistorted, &registered);
        waitKey(); // TODO: Ask for this function
        listener.release(frames);
    }

    dev->stop();
    dev->close();

    delete registration;
    return application.exec();
}

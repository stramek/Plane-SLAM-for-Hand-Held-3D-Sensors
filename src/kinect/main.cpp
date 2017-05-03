//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "include/kinect/main.h"

bool programFinished = false;

Freenect2 freenect2;
Freenect2Device *dev = nullptr;
PacketPipeline *pipeline = nullptr;

int main(int argc, char **argv) {
    QApplication application(argc, argv);
    glutInit(&argc, argv);
    QGLVisualizer visualizer;
    visualizer.setWindowTitle("Kinect pointcloud");
    visualizer.show();

    quitIfDeviceNotConnected();
    openDevice();

    SyncMultiFrameListener listener(Frame::Color | Frame::Depth);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    FrameMap frames;
    Registration *registration = new Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    Frame undistorted(512, 424, 4), registered(512, 424, 4);

    while (!programFinished) {
        listener.waitForNewFrame(frames);
        Frame *rgb = frames[Frame::Color];
        Frame *depth = frames[Frame::Depth];

        registration->apply(rgb, depth, &undistorted, &registered, true);
        visualizer.updateCloud(registration, &undistorted, &registered);

        listener.release(frames);
//        this_thread::sleep_for(chrono::milliseconds(100));
        waitKey(); // TODO: Ask for this function and QVisualizer keyPressEvent override
    }

    dev->stop();
    dev->close();

    delete registration;
    return application.exec();
}

void quitIfDeviceNotConnected() {
    if (freenect2.enumerateDevices() == 0) {
        throw runtime_error("Didn't find any kinect.");
    }
}

void openDevice() {
    pipeline = new OpenCLPacketPipeline();
    string serial = freenect2.getDefaultDeviceSerialNumber();
    dev = freenect2.openDevice(serial, pipeline);
    if (dev == 0) {
        throw runtime_error("Failure opening device!");
    }
}
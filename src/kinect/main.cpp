//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "include/kinect/main.h"

Freenect2 freenect2;
Freenect2Device *dev = nullptr;
PacketPipeline *pipeline = nullptr;

int main(int argc, char **argv) {
    QApplication application(argc, argv);
    glutInit(&argc, argv);
    QGLVisualizer visualizer;
    //visualizer.setWindowTitle("Kinect pointcloud");
    //visualizer.show();

    quitIfDeviceNotConnected();
    openDevice();

    SyncMultiFrameListener listener(Frame::Color | Frame::Depth);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    FrameMap frames;
    Registration *registration = new Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    Frame undistorted(512, 424, 4), registered(512, 424, 4);


    vector<Plane> planeVectorPreviousFrame;
    vector<Plane> planeVectorCurrentFrame;
    vector<pair<Plane, Plane>> similarPlanes;
    const int AREA_SIZE = 11; // odd number
    const int NUMBER_OF_POINTS = 50;
    if (AREA_SIZE % 2 == 0) throw runtime_error("AREA_SIZE needs to be odd number");


    while (!visualizer.isProgramFinished()) {
        planeVectorPreviousFrame.clear();
        copy(planeVectorCurrentFrame.begin(), planeVectorCurrentFrame.end(),
             back_inserter(planeVectorPreviousFrame));
        planeVectorCurrentFrame.clear();


        listener.waitForNewFrame(frames);
        Frame *rgb = frames[Frame::Color];
        Frame *depth = frames[Frame::Depth];

        registration->apply(rgb, depth, &undistorted, &registered, true);

        planeUtils::fillPlaneVector(NUMBER_OF_POINTS, AREA_SIZE, &planeVectorCurrentFrame,
                            &planeVectorPreviousFrame, 0.2, registration, &undistorted, &registered);

        //visualizer.updateCloud(registration, &undistorted, &registered);
        planeUtils::mergePlanes(planeVectorCurrentFrame);
        similarPlanes = planeUtils::getSimilarPlanes(planeVectorPreviousFrame, planeVectorCurrentFrame);
        planeUtils::filterPairsByAngle(similarPlanes);
        cout<<"Similar planes: "<<similarPlanes.size()<<endl;
        //planeUtils::visualizeSimilarPlanes(similarPlanes, imagePair1.getRgb(), imagePair2.getRgb());

//        utils::generateOctoMap("Kinect", visualizer.getPointCloud(), 0.01);

        listener.release(frames);

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
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

    if (KINECT_MODE == SHOW_POINTCLOUD) {
        visualizer.setWindowTitle("Kinect pointcloud");
        visualizer.show();
    }

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
    const int AREA_SIZE = 37; // odd number
    const int NUMBER_OF_POINTS = 200;
    if (AREA_SIZE % 2 == 0) throw runtime_error("AREA_SIZE needs to be odd number");

    Mat previousFrame;
    Mat currentFrame;

    while (!visualizer.isProgramFinished()) {
        planeVectorPreviousFrame.clear();
        copy(planeVectorCurrentFrame.begin(), planeVectorCurrentFrame.end(),
             back_inserter(planeVectorPreviousFrame));
        planeVectorCurrentFrame.clear();

        listener.waitForNewFrame(frames);
        Frame *rgb = frames[Frame::Color];
        Frame *depth = frames[Frame::Depth];

        registration->apply(rgb, depth, &undistorted, &registered, true);

        make_unique<PlaneFillerBuilder>()
                ->withKinect(registration, &undistorted, &registered)
                ->withPlaneDetector(new PcaPlaneDetector())
                ->withAreaSize(AREA_SIZE)
                ->withNumberOfPoints(NUMBER_OF_POINTS)
                ->withPreviousPlanePercent(&planeVectorPreviousFrame, 0.5)
                ->build()
                ->fillVector(&planeVectorCurrentFrame);

        if (KINECT_MODE == SHOW_POINTCLOUD) {
            visualizer.updateCloud(registration, &undistorted, &registered);
            waitKey(1);
        }

        planeUtils::mergePlanes(planeVectorCurrentFrame);
        similarPlanes = planeUtils::getSimilarPlanes(planeVectorPreviousFrame, planeVectorCurrentFrame);

        cout << "Similar planes: " << similarPlanes.size();
        if (similarPlanes.size() >= 3) {
            cout<<" OK!"<<endl;
        } else {
            cout<<" ZA MALO PLASZCZYZN!"<<endl;
        }

        if (KINECT_MODE == VISUALIZE_SIMILAR_PLANES) {
            previousFrame = currentFrame.clone();
            currentFrame = planeUtils::getRGBFrameMat(registration, &undistorted, &registered);

            if (!previousFrame.empty()) {
                planeUtils::visualizeSimilarPlanes(similarPlanes, previousFrame, currentFrame);
            }
        }

        //utils::generateOctoMap("Kinect", visualizer.getPointCloud().getPoints3D(), 0.01);

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
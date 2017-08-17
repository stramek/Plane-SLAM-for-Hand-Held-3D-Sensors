//
// Created by stramek on 31.05.17.
//
#include "include/kinect/KinectModule.h"

KinectModule::KinectModule() {

}

void KinectModule::setKinectFramesListener(KinectFramesListener *kinectFramesListener) {
    KinectModule::kinectFramesListener = kinectFramesListener;
    start();
}

void KinectModule::quitIfDeviceNotConnected() {
    if (freenect2.enumerateDevices() == 0) {
        throw runtime_error("Didn't find any kinect.");
    }
}

void KinectModule::openDevice() {
    pipeline = new OpenCLPacketPipeline();
    string serial = freenect2.getDefaultDeviceSerialNumber();
    dev = freenect2.openDevice(serial, pipeline);
    if (dev == 0) {
        throw runtime_error("Failure opening device!");
    }
}

void KinectModule::start() {
    quitIfDeviceNotConnected();
    openDevice();

    SyncMultiFrameListener listener(Frame::Color | Frame::Depth);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    FrameMap frames;
    registration = new Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    Frame undistorted(512, 424, 4), registered(512, 424, 4);

    while (!finishedProgram) {
        copyPlanesToPreviousFrames();

        listener.waitForNewFrame(frames);
        Frame *rgb = frames[Frame::Color];
        Frame *depth = frames[Frame::Depth];

        registration->apply(rgb, depth, &undistorted, &registered, true);

        KinectFrames kinectFrames(&undistorted, &registered);
        kinectFramesListener->onFramesChange(kinectFrames);
        listener.release(frames);

        mergePlanes(planeDetector);
        lastFrameValid = planeUtils::arePlanesValid(planeVectorCurrentFrame);
        //if (lastFrameValid) {
            calculateSimilarPlanes();
        //} else {
          //  similarPlanes.clear();
        //}
    }

    dev->stop();
    dev->close();

    delete registration;
}

void KinectModule::setFinishedProgram(bool finishedProgram) {
    KinectModule::finishedProgram = finishedProgram;
}

Registration *KinectModule::getRegistration() const {
    return registration;
}

void KinectModule::copyPlanesToPreviousFrames() {
    planeVectorPreviousFrame.clear();
    copy(planeVectorCurrentFrame.begin(), planeVectorCurrentFrame.end(), back_inserter(planeVectorPreviousFrame));
    planeVectorCurrentFrame.clear();
}

void KinectModule::notifyNumberOfSimilarPlanes() {
    cout << "Found " << similarPlanes.size() << " planes.";
    if (similarPlanes.size() >= 3) {
        cout << " Ok!" << endl;
    } else {
        cout << " Not too much..." << endl;
    }
}

vector<Plane> &KinectModule::getPlaneVectorPreviousFrame() {
    return planeVectorPreviousFrame;
}

vector<Plane> &KinectModule::getPlaneVectorCurrentFrame() {
    return planeVectorCurrentFrame;
}

void KinectModule::calculateSimilarPlanes() {
    similarPlanes = planeUtils::getSimilarPlanes(planeVectorPreviousFrame, planeVectorCurrentFrame);
}

void KinectModule::mergePlanes(PlaneDetector *planeDetector) {
    planeUtils::mergePlanes(planeVectorCurrentFrame, planeDetector);
}

void KinectModule::visualizePlanes(KinectFrames &kinectFrames) {
    if (similarPlanes.size() >= 3) {
        circle(currentFrame, Point(30, 30), 15, Scalar(0, 255, 0), CV_FILLED);
    } else {
        circle(currentFrame, Point(30, 30), 15, Scalar(0, 0, 255), CV_FILLED);
    }
    previousFrame = currentFrame.clone();
    currentFrame = planeUtils::getRGBFrameMat(getRegistration(), kinectFrames.getUndistorted(),
                                              kinectFrames.getRegistered());

    if (!previousFrame.empty()) {
        planeUtils::visualizeSimilarPlanes(similarPlanes, previousFrame, currentFrame);
    }

}

void KinectModule::setPlaneDetector(PlaneDetector *planeDetector) {
    KinectModule::planeDetector = planeDetector;
}

//
// Created by stramek on 31.05.17.
//
#include "include/kinect/KinectModule.h"

KinectModule::KinectModule() {

}

void KinectModule::setKinectFramesListener(KinectFramesListener *kinectFramesListener) {
    KinectModule::kinectFramesListener = kinectFramesListener;
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
        listener.waitForNewFrame(frames);
        Frame *rgb = frames[Frame::Color];
        Frame *depth = frames[Frame::Depth];

        registration->apply(rgb, depth, &undistorted, &registered, true);

        KinectFrames kinectFrames(&undistorted, &registered);
        kinectFramesListener->onFramesChange(kinectFrames);

        listener.release(frames);
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

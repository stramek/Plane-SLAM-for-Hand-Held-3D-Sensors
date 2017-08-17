//
// Created by stramek on 31.05.17.
//

#ifndef PROJEKTMAGISTERSKI_KINECTMODULE_H
#define PROJEKTMAGISTERSKI_KINECTMODULE_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <functional>
#include "include/kinect/KinectFrames.h"
#include <opencv2/opencv.hpp>
#include "include/models/Plane.h"
#include "include/utils/planeUtils.h"
#include <thread>

using namespace libfreenect2;
using namespace std;

class KinectModule {
public:

    class KinectFramesListener {
    public:
        typedef function<void(KinectFrames &)> KinectFramesCallback;

    public:
        KinectFramesListener(KinectFramesCallback kinectFramesCallback) : callback(kinectFramesCallback) {}

        void onFramesChange(KinectFrames &kinectFrames) {
            callback(kinectFrames);
        }

    private:
        KinectFramesCallback callback;
    };

    KinectModule();
    void setKinectFramesListener(KinectFramesListener *kinectFramesListener);
    void setFinishedProgram(bool finishedProgram);
    vector<Plane> &getPlaneVectorPreviousFrame();
    vector<Plane> &getPlaneVectorCurrentFrame();
    Registration *getRegistration() const;
    void visualizePlanes(KinectFrames &kinectFrames);
    void notifyNumberOfSimilarPlanes();
    void setPlaneDetector(PlaneDetector *planeDetector);

private:
    Freenect2 freenect2;
    Freenect2Device *dev = nullptr;
    PacketPipeline *pipeline = nullptr;
    KinectFramesListener *kinectFramesListener = nullptr;
    Registration *registration = nullptr;
    PlaneDetector *planeDetector = nullptr;

    vector<Plane> planeVectorPreviousFrame;
    vector<Plane> planeVectorCurrentFrame;
    vector<pair<Plane, Plane>> similarPlanes;

    Mat previousFrame;
    Mat currentFrame;

    bool finishedProgram = false;
    bool lastFrameValid = false;

    void quitIfDeviceNotConnected();
    void copyPlanesToPreviousFrames();
    void openDevice();
    void calculateSimilarPlanes();
    void mergePlanes(PlaneDetector *planeDetector);
    void start();

    void setMergePlaneDetector(PlaneDetector *planeDetector);
};


#endif //PROJEKTMAGISTERSKI_KINECTMODULE_H

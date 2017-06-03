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

using namespace libfreenect2;
using namespace std;

class KinectModule {
public:

    class KinectFramesListener {
    public:
        typedef function<void(KinectFrames&)> KinectFramesCallback;

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

    void start();

    Registration *getRegistration() const;

private:
    Freenect2 freenect2;
    Freenect2Device *dev = nullptr;
    PacketPipeline *pipeline = nullptr;
    KinectFramesListener *kinectFramesListener = nullptr;
    Registration *registration = nullptr;

    bool finishedProgram = false;

    void quitIfDeviceNotConnected();
    void openDevice();

};


#endif //PROJEKTMAGISTERSKI_KINECTMODULE_H

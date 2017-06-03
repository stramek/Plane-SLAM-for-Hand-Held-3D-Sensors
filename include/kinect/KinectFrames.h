//
// Created by stramek on 31.05.17.
//

#ifndef PROJEKTMAGISTERSKI_KINECTFRAMES_H
#define PROJEKTMAGISTERSKI_KINECTFRAMES_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <iostream>
#include <string>

using namespace libfreenect2;
using namespace std;

class KinectFrames {
public:
    KinectFrames(Frame *undistorted, Frame *registered);

    Frame *getUndistorted() const;

    Frame *getRegistered() const;

private:
    Frame *undistorted = nullptr;
    Frame *registered = nullptr;
};

#endif //PROJEKTMAGISTERSKI_KINECTFRAMES_H

//
// Created by stramek on 31.05.17.
//

#include "include/kinect/KinectFrames.h"

KinectFrames::KinectFrames(Frame *undistorted, Frame *registered) : undistorted(undistorted), registered(registered) {}

Frame *KinectFrames::getUndistorted() const {
    return undistorted;
}

Frame *KinectFrames::getRegistered() const {
    return registered;
}

//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "include/kinect/main.h"

int main(int argc, char **argv) {
    QApplication application(argc, argv);
    glutInit(&argc, argv);

    KinectModule kinectModule;
    kinectModule.setKinectFramesListener(new KinectModule::KinectFramesListener(
            [&](KinectFrames &kinectFrames) {

                make_unique<PlaneFillerBuilder>()
                        ->withKinect(kinectModule.getRegistration(), kinectFrames.getUndistorted(),
                                     kinectFrames.getRegistered())
                        ->withPlaneDetector(new PcaPlaneDetector())
                        ->withAreaSize(31)
                        ->withNumberOfPoints(400)
                        ->withPreviousPlanePercent(&kinectModule.getPlaneVectorPreviousFrame(), 0.5)
                        ->build()
                        ->fillVector(&kinectModule.getPlaneVectorCurrentFrame());

                kinectModule.visualizePlanes(kinectFrames);
                kinectModule.notifyNumberOfSimilarPlanes();
            }
    ));

    return application.exec();
}
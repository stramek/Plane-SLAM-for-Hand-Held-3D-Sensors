//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "include/kinect/main.h"

vector<Plane> planeVectorPreviousFrame;
vector<Plane> planeVectorCurrentFrame;
vector<pair<Plane, Plane>> similarPlanes;

Mat previousFrame;
Mat currentFrame;

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
                        ->withAreaSize(35)
                        ->withNumberOfPoints(300)
                        ->withPreviousPlanePercent(&planeVectorPreviousFrame, 0.5)
                        ->build()
                        ->fillVector(&planeVectorCurrentFrame);

                planeUtils::mergePlanes(planeVectorCurrentFrame);
                similarPlanes = planeUtils::getSimilarPlanes(planeVectorPreviousFrame, planeVectorCurrentFrame);

                copyPlanesToPreviousFrames();
                visualizePlanes(kinectModule, kinectFrames);
                notifyNumberOfSimilarPlanes();
            }
    ));
    kinectModule.start();
    return application.exec();
}

void copyPlanesToPreviousFrames() {
    planeVectorPreviousFrame.clear();
    copy(planeVectorCurrentFrame.begin(), planeVectorCurrentFrame.end(), back_inserter(planeVectorPreviousFrame));
    planeVectorCurrentFrame.clear();
}

void visualizePlanes(KinectModule &kinectModule, KinectFrames &kinectFrames) {
    previousFrame = currentFrame.clone();
    currentFrame = planeUtils::getRGBFrameMat(kinectModule.getRegistration(), kinectFrames.getUndistorted(),
                                              kinectFrames.getRegistered());

    if (!previousFrame.empty()) {
        planeUtils::visualizeSimilarPlanes(similarPlanes, previousFrame, currentFrame);
    }
}

void notifyNumberOfSimilarPlanes() {
    cout << "Found " << similarPlanes.size() << " planes.";
    if (similarPlanes.size() >= 3) {
        cout << " Ok!" << endl;
    } else {
        cout << " Not too much..." << endl;
    }
}
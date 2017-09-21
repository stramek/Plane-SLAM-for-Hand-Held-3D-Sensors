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
    QGLVisualizer visualizer;

    remove("trajectories/trajectory_SAME_POINT.txt");
    utils::createOctoMap("Kinect", 0.01);

    PlaneDetector* planeDetectorMethod = new PcaPlaneDetector();
    KinectModule kinectModule;
    kinectModule.setPlaneDetector(planeDetectorMethod);

    kinectModule.setKinectFramesListener(new KinectModule::KinectFramesListener(
            [&](KinectFrames &kinectFrames, bool didLocationChanged, const PosOrient& lastKnownPosition) {

                make_unique<PlaneFillerBuilder>()
                        ->withKinect(kinectModule.getRegistration(), kinectFrames.getUndistorted(),
                                     kinectFrames.getRegistered())
                        ->withPlaneDetector(planeDetectorMethod)
                        ->withAreaSize(35)
                        ->withNumberOfPoints(800)
                        ->withPreviousPlanePercent(&kinectModule.getPlaneVectorPreviousFrame(), 0.5)
                        ->build()
                        ->fillVector(&kinectModule.getPlaneVectorCurrentFrame());

                if (didLocationChanged) {
                    //visualizer.updateCloud(kinectModule.getRegistration(), kinectFrames.getUndistorted(), kinectFrames.getRegistered(), lastKnownPosition);
                    utils::appendTrajectoryRecord("trajectories/trajectory_SAME_POINT.txt", lastKnownPosition);
                    //utils::updateOctoMap("Kinect", visualizer.getPointCloud().getPoints3D());
                    cout<<"*********"<<endl;
                    lastKnownPosition.print();
                    cout<<"*********"<<endl;
                }

                kinectModule.visualizePlanes(kinectFrames);
            }
    ));

    return application.exec();
}
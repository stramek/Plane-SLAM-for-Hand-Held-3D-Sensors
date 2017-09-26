//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "include/dataset/main.h"
#include <Eigen/Dense>
#include <include/planeG2O/GlobalG2oMap.h>

int main(int argc, char **argv) {
    QApplication application(argc, argv);
    glutInit(&argc, argv);
    QGLVisualizer visualizer;

    ImageLoader imageLoader(1500);
    imageLoader.setCurrentPhoto(1);

    vector<Plane> planeVectorPreviousFrame;
    vector<Plane> planeVectorCurrentFrame;
    vector<pair<Plane, Plane>> similarPlanes;
    vector<PosOrient> idealSlamPositions;
    Mat previousRgbImage;

    PosOrient lastPosOrient;

    const bool visualize = false;

    //utils::loadDatasetPositions(idealSlamPositions);
    ofstream trajectoryFile;

    string currentDate = utils::getCurrentDate();
    GlobalG2oMap globalG2oMap;

//    utils::createOctoMap("Dataset", 0.02);


    int numberOfIterations = 2;
    for (int i = 0; i < numberOfIterations; ++i) {
        trajectoryFile.open("trajectories/trajectory_" + currentDate + ".txt", std::ios_base::app);

        ImagePair currentFrame;
        if (i == 0) {
            currentFrame = imageLoader.getNextPair();
        } else {
            currentFrame = imageLoader.getNextPair(118);
        }


        make_unique<PlaneFillerBuilder>()
                ->withDataset(&currentFrame)
                ->withPlaneDetector(new PcaPlaneDetector())
                ->withAreaSize(41)
                ->withNumberOfPoints(600)
                ->withPreviousPlanePercent(&planeVectorPreviousFrame, 0.5)
                ->build()
                ->fillVector(&planeVectorCurrentFrame);

        planeUtils::mergePlanes(planeVectorCurrentFrame, new PcaPlaneDetector());

        globalG2oMap.addNewFrames(planeVectorCurrentFrame);

        PosOrient posOrient = globalG2oMap.getLastPosOrient();
        Vector3d position = posOrient.getPosition();
        Quaterniond q  = posOrient.getQuaternion();

        trajectoryFile << position[0] << " " << position[1] << " " << position[2] << " " << q.w() << " " << q.x()
                       << " " << q.y() << " " << q.z() << "\n";

        cout<<">>>>> Iteration "<<i+1<<" of " <<numberOfIterations<<" finished."<<endl;
        if (i != 0) {
            globalG2oMap.printLastPoseOrient();
            visualizer.updateCloud(currentFrame.getRgb(), currentFrame.getDepth(), globalG2oMap.getLastPosOrient());
            if (lastPosOrient.getPosition()[0] != globalG2oMap.getLastPosOrient().getPosition()[0]
                && lastPosOrient.getPosition()[1] != globalG2oMap.getLastPosOrient().getPosition()[1]
                && lastPosOrient.getPosition()[2] != globalG2oMap.getLastPosOrient().getPosition()[2]) {
//                utils::updateOctoMap("Dataset", visualizer.getGlobalDatasetPointCloud().getPoints3D());
                cout<<"Not same"<<endl;
            } else {
                cout<<"Same orient"<<endl;
            }
            lastPosOrient = globalG2oMap.getLastPosOrient();
            planeUtils::visualizeSimilarPlanes(globalG2oMap.getMatchedPlanes(), previousRgbImage, currentFrame.getRgb());
            waitKey();
        }
        previousRgbImage = currentFrame.getRgb().clone();
        trajectoryFile.close();
    }

    globalG2oMap.saveTrajectoryToFile();

    cout<<"Done XD"<<endl;

    return application.exec();
}

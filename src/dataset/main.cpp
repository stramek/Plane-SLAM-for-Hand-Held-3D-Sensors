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

    ImageLoader imageLoader(500);
    imageLoader.setCurrentPhoto(1);

    vector<Plane> planeVectorPreviousFrame;
    vector<Plane> planeVectorCurrentFrame;
    vector<pair<Plane, Plane>> similarPlanes;
    vector<PosOrient> idealSlamPositions;
    Mat previousRgbImage;

    const bool visualize = false;

    //utils::loadDatasetPositions(idealSlamPositions);
    ofstream trajectoryFile;

    string currentDate = utils::getCurrentDate();
    GlobalG2oMap globalG2oMap;

    int numberOfIterations = 500;
    for (int i = 0; i < numberOfIterations; ++i) {
        trajectoryFile.open("trajectories/trajectory_" + currentDate + ".txt", std::ios_base::app);

        ImagePair currentFrame = imageLoader.getNextPair();

        make_unique<PlaneFillerBuilder>()
                ->withDataset(&currentFrame)
                ->withPlaneDetector(new PcaPlaneDetector())
                ->withAreaSize(35)
                ->withNumberOfPoints(2000)
                ->withPreviousPlanePercent(&planeVectorPreviousFrame, 0.5)
                ->build()
                ->fillVector(&planeVectorCurrentFrame);

        planeUtils::mergePlanes(planeVectorCurrentFrame, new PcaPlaneDetector());

        if (planeUtils::arePlanesValid(planeVectorCurrentFrame)) {
            globalG2oMap.addNewFrames(planeVectorCurrentFrame);
        } else {
            cout<<"*********************************************"<<endl;
            cout<<"Planes are not valid! SKIPPING FRAME "<<i<<"!"<<endl;
            cout<<"*********************************************"<<endl;
        }

        PosOrient posOrient = globalG2oMap.getLastPosOrient();
        Vector3d position = posOrient.getPosition();
        Quaterniond q  = posOrient.getQuaternion();

        trajectoryFile << position[0] << " " << position[1] << " " << position[2] << " " << q.w() << " " << q.x()
                       << " " << q.y() << " " << q.z() << "\n";

        cout<<">>>>> Iteration "<<i+1<<" of " <<numberOfIterations<<" finished."<<endl;
        if (i != 0) {
            globalG2oMap.printLastPoseOrient();
            planeUtils::visualizeSimilarPlanes(globalG2oMap.getMatchedPlanes(), previousRgbImage, currentFrame.getRgb());
            waitKey();
        }
        previousRgbImage = currentFrame.getRgb().clone();


/*        if (planeUtils::arePlanesValid(planeVectorCurrentFrame)) {
            cout<<"Frame number "<<i + 1<<" is valid!"<<endl;
            if (!planeVectorPreviousFrame.empty()) {
                similarPlanes = planeUtils::getSimilarPlanes(planeVectorPreviousFrame, planeVectorCurrentFrame);
                planeG2o.ComputeCameraPos(similarPlanes);
                cout << "Frame " << i << "-" << i + 1 << " found: " << similarPlanes.size() << " similar planes." << endl;
                cout << endl << "Ideal slam position" << endl;
                idealSlamPositions.at((unsigned int) (40)).print();
                cout << endl;
                if (visualize) {
                    planeUtils::visualizeSimilarPlanes(similarPlanes, previousRgbImage, currentFrame.getRgb());
                    waitKey();
                }
            }
            utils::movePlanesToPreviousVector(planeVectorPreviousFrame, planeVectorCurrentFrame);
            if (visualize) previousRgbImage = currentFrame.getRgb().clone();
        } else {
            cout<<"Frame number"<<i + 1<<"is NOT valid!";
        }*/
        trajectoryFile.close();
    }

    globalG2oMap.saveTrajectoryToFile();

    cout<<"Done XD"<<endl;

    return application.exec();
}

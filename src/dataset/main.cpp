//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "include/dataset/main.h"
#include <Eigen/Dense>

int main(int argc, char **argv) {
    QApplication application(argc, argv);
    glutInit(&argc, argv);

    Eigen::Vector3d point1 = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d point2 = Eigen::Vector3d(0, 1, 2);
    Eigen::Vector3d point3 = Eigen::Vector3d(0, 3, 4);
    Plane plane = Plane(point1, point2, point3, ImageCoords());

    Eigen::Vector3d position = Eigen::Vector3d(1, 2, 3);
    Eigen::Vector4d orientation = Eigen::Vector4d(0, 0, 0.707, 0.707);

    PosOrient posOrient = PosOrient(position, orientation);
    Plane newPlane = plane.getPlaneSeenFromGlobalCamera(posOrient);

    /*ImageLoader imageLoader(50);

    vector<Plane> planeVectorPreviousFrame;
    vector<Plane> planeVectorCurrentFrame;
    vector<pair<Plane, Plane>> similarPlanes;
    vector<PosOrient> idealSlamPositions;
    Mat previousRgbImage;

    const bool visualize = true;


    utils::loadDatasetPositions(idealSlamPositions);

    PlaneG2oModule &planeG2o = PlaneG2oModule::getInstance();

    for (int i = 0; i < 2; ++i) {
        ImagePair currentFrame = imageLoader.getNextPair(40);

        make_unique<PlaneFillerBuilder>()
                ->withDataset(&currentFrame)
                ->withPlaneDetector(new PcaPlaneDetector())
                ->withAreaSize(35)
                ->withNumberOfPoints(600)
                ->withPreviousPlanePercent(&planeVectorPreviousFrame, 0.0)
                ->build()
                ->fillVector(&planeVectorCurrentFrame);

        planeUtils::mergePlanes(planeVectorCurrentFrame, new PcaPlaneDetector());

        if (planeUtils::arePlanesValid(planeVectorCurrentFrame)) {
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
        }
    }*/

    return application.exec();
}

//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include <include/planeG2O/MatchPlanesG2o.h>
#include "include/dataset/main.h"

int main(int argc, char **argv) {
    QApplication application(argc, argv);
    glutInit(&argc, argv);

    ImageLoader imageLoader(50);

    vector<Plane> planeVectorPreviousFrame;
    vector<Plane> planeVectorCurrentFrame;
    vector<pair<Plane, Plane>> similarPlanes;
    vector<PosOrient> idealSlamPositions;
    Mat previousRgbImage;

    const bool visualize = true;


    utils::loadDatasetPositions(idealSlamPositions);

    PlaneG2oModule planeG2o;

    //test eigen quaternion rotation

    Eigen::Quaterniond q(0.7071, 0, 0, 0.7071);
    Eigen::Quaterniond v(0, 1, 0, 0);
    Eigen::Quaterniond v1 = q*v*q.conjugate();
    cout<< "Vector after rotation v1: " << v1.w() << " " << v1.x() << " " << v1.y() << " " << v1.z() << endl;

    Eigen::Vector3d vr(1, 0, 0);
    vr = q.toRotationMatrix()*vr;
    cout<< "Vector after rotation v2: " << vr.x() << " " << vr.y() << " " << vr.z() << endl;

    //

    for (int i = 0; i < 50; ++i) {
        ImagePair currentFrame = imageLoader.getNextPair();

        make_unique<PlaneFillerBuilder>()
                ->withDataset(&currentFrame)
                ->withPlaneDetector(new PcaPlaneDetector())
                ->withAreaSize(35)
                ->withNumberOfPoints(200)
                ->withPreviousPlanePercent(&planeVectorPreviousFrame, 0.0)
                ->build()
                ->fillVector(&planeVectorCurrentFrame);

        planeUtils::mergePlanes(planeVectorCurrentFrame, new PcaPlaneDetector());

        if (planeUtils::arePlanesValid(planeVectorCurrentFrame)) {
            cout<<"Frame number "<<i + 1<<" is valid!"<<endl;
            if (!planeVectorPreviousFrame.empty()) {
                //similarPlanes = planeUtils::getSimilarPlanes(planeVectorPreviousFrame, planeVectorCurrentFrame);
                MatchPlanesG2o matchPlanesG2o;
                similarPlanes = matchPlanesG2o.getSimilarPlanes(planeVectorPreviousFrame, planeVectorCurrentFrame);
                std::cout << "After match" << std::endl;
/*                planeG2o.ComputeCameraPos(similarPlanes);
                cout << "Frame " << i << "-" << i + 1 << " found: " << similarPlanes.size() << " similar planes." << endl;
                cout << endl << "Ideal slam position" << endl;
                idealSlamPositions.at((unsigned int) (40)).print();
                cout << endl;*/
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
    }

    return application.exec();
}

//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "include/dataset/main.h"

int main(int argc, char **argv) {

    //some testing stuff
    std::vector<Plane> planesVec;
    Eigen::Vector3f v1, v2, v3, v4;
    v1 << 1, 0, 0;
    v2 << 1, 1, 0;
    v3 << 0, 1, 0;
    v4 << -1, 0, 0;
    Plane p1(v1, 0), p2(v2, 5), p3(v3, 0), p4(v4, 0);
    planesVec.push_back(p1);
    planesVec.push_back(p2);
    planesVec.push_back(p3);
    planesVec.push_back(p4);
    std::vector<Cluster> clustersVec;
    std::vector<std::unordered_set<int>> vecEachClusterPlanes;
    Clustering::getClustersAfterThreshold(46, planesVec, vecEachClusterPlanes);

    std::cout<<std::endl;
    for(auto i : vecEachClusterPlanes){
        for(auto j : i){
            std::cout<<j<<" ";
        }
        std::cout<<std::endl;
    }




    /////////////////////////////////////

/*    QApplication application(argc, argv);
    glutInit(&argc, argv);

    QGLVisualizer visualizer;
    visualizer.setWindowTitle("Dataset viewer");
    visualizer.setPHCPModel(PHCP_MODEL);
    visualizer.show();

    ImageLoader imageLoader(50);
    ImagePair imagePair;

    vector<Plane> planeVectorPreviousFrame;
    vector<Plane> planeVectorCurrentFrame;
    vector<pair<Plane, Plane>> similarPlanes;

    const int AREA_SIZE = 21; // odd number
    const int NUMBER_OF_POINTS = 50;
    if (AREA_SIZE % 2 == 0) throw runtime_error("AREA_SIZE needs to be odd number");

    imagePair = imageLoader.getNextPair();
    utils::fillPlaneVector(NUMBER_OF_POINTS, AREA_SIZE, imagePair, planeVectorPreviousFrame);
    imagePair = imageLoader.getNextPair();
    utils::fillPlaneVector(NUMBER_OF_POINTS, AREA_SIZE, imagePair, planeVectorCurrentFrame);
    similarPlanes = utils::getSimilarPlanes(planeVectorPreviousFrame, planeVectorCurrentFrame);

    visualizer.updateCloud(imagePair.getRgb(), imagePair.getDepth());

    return application.exec();*/
    return 0;
}

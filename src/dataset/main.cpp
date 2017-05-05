//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "include/dataset/main.h"

int main(int argc, char **argv) {

    QApplication application(argc, argv);
    glutInit(&argc, argv);

    QGLVisualizer visualizer;
    visualizer.setWindowTitle("Dataset viewer");
    visualizer.setPHCPModel(PHCP_MODEL);
    visualizer.show();

    ImageLoader imageLoader(50);

    vector<Plane> planeVectorPreviousFrame;
    vector<Plane> planeVectorCurrentFrame;
    vector<pair<Plane, Plane>> similarPlanes;

    const int AREA_SIZE = 21; // odd number
    const int NUMBER_OF_POINTS = 10;
    if (AREA_SIZE % 2 == 0) throw runtime_error("AREA_SIZE needs to be odd number");

    ImagePair imagePair1 = imageLoader.getNextPair();
    utils::fillPlaneVector(NUMBER_OF_POINTS, AREA_SIZE, imagePair1, planeVectorPreviousFrame);

    vector<vector<Plane>> clusteredPLanes;
    Clustering::getClusteredPlaneGroup(planeVectorPreviousFrame, clusteredPLanes);
    int i = 0;
    std::cout<<std::endl;
    for (auto singleCluster : clusteredPLanes){
        ++i;
        std::cout<<"Cluster number " << i << std::endl;
        for(auto planeInCluster : singleCluster){
            putText(imagePair1.getRgb(), to_string(i), Point(planeInCluster.getImageCoords().getCenterX(), planeInCluster.getImageCoords().getCenterY()),FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            std::cout<<"A: "<<planeInCluster.getA() << " B: "<<planeInCluster.getB() << " C: "<<planeInCluster.getC() << " D: "<<planeInCluster.getD() << std::endl;
        }
        std::cout<<std::endl;

    }

    utils::mergePlanes(planeVectorPreviousFrame);
    for(int i=0;i<planeVectorPreviousFrame.size();++i){
        std::cout<<"Cluster number " << i + 1<< std::endl;
        std::cout<<"A: "<<planeVectorPreviousFrame.at(i).getA() << " B: "<<planeVectorPreviousFrame.at(i).getB() << " C: "<<planeVectorPreviousFrame.at(i).getC() << " D: "<<planeVectorPreviousFrame.at(i).getD() << std::endl;
    }

    visualizer.updateCloud(imagePair1.getRgb(), imagePair1.getDepth());
    imshow("First", imagePair1.getRgb());


    return application.exec();
    return 0;
}

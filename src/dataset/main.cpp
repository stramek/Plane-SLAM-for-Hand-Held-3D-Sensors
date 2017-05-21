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

    QGLVisualizer visualizerT;
    visualizerT.setWindowTitle("Dataset viewer reverse normal");
    visualizerT.setPHCPModel(PHCP_MODEL);
    visualizerT.show();

    //while (true) {

        ImageLoader imageLoader(50);

        vector<Plane> planeVectorPreviousFrame;
        vector<Plane> planeVectorCurrentFrame;
        vector<pair<Plane, Plane>> similarPlanes;

        const int AREA_SIZE = 21; // odd number
        const int NUMBER_OF_POINTS = 50;
        if (AREA_SIZE % 2 == 0) throw runtime_error("AREA_SIZE needs to be odd number");

        ImagePair imagePair1 = imageLoader.getNextPair();
        planeUtils::fillPlaneVector(NUMBER_OF_POINTS, AREA_SIZE, imagePair1, &planeVectorPreviousFrame);
        std::cout<<std::endl;
        ImagePair imagePair2(imagePair1.getRgb().clone(), imagePair1.getDepth().clone());
        planeUtils::fillPlaneVector(NUMBER_OF_POINTS, AREA_SIZE, imagePair2, &planeVectorCurrentFrame,
                                    &planeVectorPreviousFrame, 2.0f, true, true);


        planeUtils::displayClusteredPlanes(imagePair2, planeVectorCurrentFrame);


        //planeUtils::mergePlanes(planeVectorPreviousFrame);
        //planeUtils::mergePlanes(planeVectorCurrentFrame);
       // similarPlanes = planeUtils::getSimilarPlanes(planeVectorPreviousFrame, planeVectorCurrentFrame);

      //  planeUtils::filterPairsByAngle(similarPlanes);
      //  planeUtils::visualizeSimilarPlanes(similarPlanes, imagePair1.getRgb(), imagePair2.getRgb());
        visualizer.updateCloud(imagePair1.getRgb(), imagePair1.getDepth());
        visualizer.updatePlanes(planeVectorPreviousFrame);

        visualizerT.updateCloud(imagePair2.getRgb(), imagePair2.getDepth());
        visualizerT.updatePlanes(planeVectorCurrentFrame);

        waitKey();

        //utils::generateOctoMap("Dataset", visualizer.getPointCloud());
    //}


    return application.exec();
}

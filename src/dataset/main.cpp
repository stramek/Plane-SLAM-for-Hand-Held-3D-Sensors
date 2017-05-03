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
    ImagePair imagePair;

    vector<Plane> planeVectorPreviousFrame;
    vector<Plane> planeVectorCurrentFrame;
    vector<pair<Plane, Plane>> similarPlanes;

    const int AREA_SIZE = 21; // odd number
    const int NUMBER_OF_POINTS = 10;
    if (AREA_SIZE % 2 == 0) throw runtime_error("AREA_SIZE needs to be odd number");

    imagePair = imageLoader.getNextPair();
    utils::fillPlaneVector(NUMBER_OF_POINTS, AREA_SIZE, imagePair, planeVectorPreviousFrame);
    imagePair = imageLoader.getNextPair();
    utils::fillPlaneVector(NUMBER_OF_POINTS, AREA_SIZE, imagePair, planeVectorCurrentFrame);
    similarPlanes = utils::getSimilarPlanes(planeVectorPreviousFrame, planeVectorCurrentFrame);

    visualizer.updateCloud(imagePair.getRgb(), imagePair.getDepth());

    return application.exec();
}

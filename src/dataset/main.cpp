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
    ImagePair imagePair2 = imageLoader.getNextPair();

    utils::fillPlaneVector(NUMBER_OF_POINTS, AREA_SIZE, imagePair2, planeVectorCurrentFrame);
    similarPlanes = utils::getSimilarPlanes(planeVectorPreviousFrame, planeVectorCurrentFrame);

    visualizer.updateCloud(imagePair1.getRgb(), imagePair1.getDepth());
//    imshow("First", imagePair1.getRgb());
//    imshow("Second", imagePair2.getRgb());

//    Size sz1 = imagePair1.getRgb().size();
//    Size sz2 = imagePair2.getRgb().size();
//    Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
//    Mat left(im3, Rect(0, 0, sz1.width, sz1.height));
//    imagePair1.getRgb().copyTo(left);
//    Mat right(im3, Rect(sz1.width, 0, sz2.width, sz2.height));
//    imagePair2.getRgb().copyTo(right);
//    imshow("Two", im3);

    return application.exec();
}

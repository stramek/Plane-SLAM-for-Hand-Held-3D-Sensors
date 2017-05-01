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
    ImagePair imagePair = imageLoader.getNextPair();

    vector<Plane> planeVector;

    const int AREA_SIZE = 21; // odd number
    const int NUMBER_OF_POINTS = 10;
    if (AREA_SIZE % 2 == 0) throw runtime_error("AREA_SIZE needs to be odd number");

    for (int iteration = 0; iteration < NUMBER_OF_POINTS; ++iteration) {
        pair<int, int> position = utils::getRandomPosition(imagePair.getDepth(), AREA_SIZE);
        ImageCoords imageCoords = ImageCoords(position, AREA_SIZE);
        Mat rgb = imagePair.getRgb();
        Mat croppedImage = rgb(Rect(imageCoords.getUpLeftX(),
                                    imageCoords.getUpLeftY(),
                                    imageCoords.getAreaSize(),
                                    imageCoords.getAreaSize()));

        vector<Vector3f> pointsVector;
        for (int i = imageCoords.getUpLeftY(); i <= imageCoords.getDownRightY(); ++i) {
            for (int j = imageCoords.getUpLeftX(); j <= imageCoords.getDownRightX(); ++j) {
                pointsVector.push_back(Vector3f(i, j, imagePair.getDepthAt(i, j)));
            }
        }

        Plane plane = PlanePca::getPlane(pointsVector, croppedImage);
        Vec3b color = plane.isValid() ? Vec3b(0, 255, 0) : Vec3b(0, 0, 255);
        for (Vector3f vector : pointsVector) {
            utils::paintPixel((Mat &) imagePair.getRgb(), vector, color);
        }
        if (plane.isValid()) {
            planeVector.push_back(plane);
        }
    }
    visualizer.updateCloud(imagePair.getRgb(), imagePair.getDepth());

    for (Plane plane1 : planeVector) {
        cout<<"Hue: "<<to_string(plane1.getColor().getHue())<<"."<<endl;
        cout<<"Sat: "<<to_string(plane1.getColor().getSaturation())<<"."<<endl;
        cout<<"Val: "<<to_string(plane1.getColor().getValue())<<"."<<endl<<endl;
    }

    return application.exec();
}

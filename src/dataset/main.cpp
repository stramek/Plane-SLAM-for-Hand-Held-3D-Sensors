//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "../../include/dataset/main.h"

int visualizerTest(int argc, char **argv) {
    QApplication application(argc, argv);
    //setlocale(LC_NUMERIC,"C");
    glutInit(&argc, argv);

    QGLVisualizer visu;

    visu.setWindowTitle("Simulator viewer");
    visu.show();

    return application.exec();
}

int main(int argc, char **argv) {

    Vector3f vec1(0, 0, 1), vec2(1, 1, 0), vec3(1, 0, 0.5);
    vector<Vector3f> pointsVector;
    pointsVector.push_back(vec1);
    pointsVector.push_back(vec2);
    pointsVector.push_back(vec3);
    Plane planeTest = PCA::getPlane(pointsVector);
    cout << planeTest.getA() << endl << planeTest.getB() << endl << planeTest.getC() << endl << planeTest.getD()
         << endl;
    //cout << PHCP_MODEL;
    //visualizerTest(argc, argv);
}

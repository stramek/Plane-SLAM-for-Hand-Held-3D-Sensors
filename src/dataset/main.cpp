//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "../../include/dataset/main.h"

int visualizerTest(int argc, char** argv){
    QApplication application(argc,argv);
    //setlocale(LC_NUMERIC,"C");
    glutInit(&argc, argv);

    QGLVisualizer visu;

    visu.setWindowTitle("Simulator viewer");
    visu.show();

    return application.exec();
}

int main(int argc, char** argv) {

    Vector3f vec1(0.5, 0, 0), vec2(1, 2, 1.4), vec3(100, -100, -101/5.0);
    vector<Vector3f> pointsVector;
    pointsVector.push_back(vec1);
    pointsVector.push_back(vec2);
    pointsVector.push_back(vec3);
    cout << pointsVector.at(0);
    Plane plane(vec1, vec2);
    //Plane planeTest(vec1, vec2);//PCA::getPlane(pointsVector);
    //cout<<planeTest.getA() << endl << planeTest.getB() << endl << planeTest.getC() << endl << planeTest.getD() << endl;
    //cout << PHCP_MODEL;
    //visualizerTest(argc, argv);
}

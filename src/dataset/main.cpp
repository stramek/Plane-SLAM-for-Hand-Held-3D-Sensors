//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "../../include/dataset/main.h"
#include "include/dataset/PlanePca.h"

void startSlides() {
    ImageLoader imageLoader(50);
    auto interval = observable<>::interval(std::chrono::milliseconds(50));
    interval
            .as_blocking()
            .subscribe([&](long value) {
                           ImagePair imagePair = imageLoader.getNextPair();
                           imshow("rgb", imagePair.getRgb());
                           imshow("depth", imagePair.getDepth());
                           waitKey(1);
                       },
                       []() {
                           printf("\nOnCompleted");
                       });
}

int main(int argc, char** argv) {
    //startSlides();

    /*QApplication application(argc,argv);
    glutInit(&argc, argv);

    QGLVisualizer visu;
    visu.setWindowTitle("Dataset viewer");
    visu.show();

    return application.exec();*/
    Vector3f vec1(0, 0, 1), vec2(1, 1, 0), vec3(1, 0, 0.5);
    vector<Vector3f> pointsVector;
    pointsVector.push_back(vec1);
    pointsVector.push_back(vec2);
    pointsVector.push_back(vec3);
    Plane plane = PlanePca::getPlane(pointsVector);
    cout<<plane.getA() << " " << plane.getB() << " " << plane.getC() << " " << plane.getD() << endl;
    return -1;
}

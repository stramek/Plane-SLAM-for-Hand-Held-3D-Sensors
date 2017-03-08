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

    QGLVisualizer visu;
    visu.setWindowTitle("Dataset viewer");
    visu.show();

    ImageLoader imageLoader(50);
    auto interval = observable<>::interval(std::chrono::milliseconds(1000));
    interval
            .as_blocking()
            .subscribe([&](long value) {
                            cout<<value<<endl;
                           ImagePair imagePair = imageLoader.getNextPair();
                            visu.updateCloud(imagePair.getRgb(), imagePair.getDepth());
                            //visu.draw();
                           //imshow("rgb", imagePair.getRgb());
                           //imshow("depth", imagePair.getDepth());
                           //waitKey(1);
                       },
                       []() {
                           printf("\nOnCompleted");
                       });
}

pair<int, int> getRandomPosition(const Mat& mat) {
    random_device rd;
    mt19937 rng(rd());
    uniform_int_distribution<int> colUni(0, mat.cols);
    uniform_int_distribution<int> rowUni(0, mat.rows);
    return pair<int, int>(rowUni(rng), colUni(rng));
}

int main(int argc, char** argv) {


    QApplication application(argc,argv);
    glutInit(&argc, argv);

    QGLVisualizer visu;
    visu.setWindowTitle("Dataset viewer");
    visu.setPHCPModel(PHCP_MODEL);
    ImageLoader imageLoader(50);
    visu.show();

    ImagePair imagePair = imageLoader.getNextPair();
    //visu.updateCloud(imagePair.getRgb(), imagePair.getDepth());


    const int areaSize = 21; // odd number
    const int numberOfPoints = 10;

    if (areaSize % 2 == 0) throw runtime_error("areaSize needs to be odd number");

    for (int z = 0; z < numberOfPoints; ++z) {
        cout<<"Iteration "<<z<<": ";

        pair<int, int> position = getRandomPosition(imagePair.getDepth());

        vector<Vector3f> pointsVector;
        for (int i = position.first - (areaSize - 1) / 2; i <= position.first + (areaSize - 1) / 2; ++i) {
            for (int j = position.second - (areaSize - 1) / 2; j <= position.second + (areaSize - 1) / 2; ++j) {
                pointsVector.push_back(Vector3f(i, j, imagePair.getDepthAt(i, j)));
            }
        }

        Plane plane = PlanePca::getPlane(pointsVector);

        if (plane.isValid()) {
            cout<<"Plane valid!"<<endl;

            Mat rgb = imagePair.getRgb();
            for (int i = position.first - (areaSize - 1) / 2; i <= position.first + (areaSize - 1) / 2; ++i) {
                for (int j = position.second - (areaSize - 1) / 2; j <= position.second + (areaSize - 1) / 2; ++j) {
                    rgb.at<Vec3b>(i,j)[0] = 0;
                    rgb.at<Vec3b>(i,j)[1] = 255;
                    rgb.at<Vec3b>(i,j)[2] = 0;
                }
            }


        } else {
            cout<<"Plane is not valid :("<<endl;

            Mat rgb = imagePair.getRgb();
            for (int i = position.first - (areaSize - 1) / 2; i <= position.first + (areaSize - 1) / 2; ++i) {
                for (int j = position.second - (areaSize - 1) / 2; j <= position.second + (areaSize - 1) / 2; ++j) {
                    rgb.at<Vec3b>(i,j)[0] = 0;
                    rgb.at<Vec3b>(i,j)[1] = 0;
                    rgb.at<Vec3b>(i,j)[2] = 255;
                }
            }
        }
    }

    visu.updateCloud(imagePair.getRgb(), imagePair.getDepth());
    return application.exec();
}

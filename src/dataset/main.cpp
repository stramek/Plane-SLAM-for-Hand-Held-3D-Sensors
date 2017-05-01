//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "include/dataset/main.h"

int main(int argc, char **argv) {
/*    QApplication application(argc, argv);
    glutInit(&argc, argv);

    QGLVisualizer visualizer;
    visualizer.setWindowTitle("Dataset viewer");
    visualizer.setPHCPModel(PHCP_MODEL);
    visualizer.show();

    ImageLoader imageLoader(50);
    ImagePair imagePair = imageLoader.getNextPair();

    const int AREA_SIZE = 21; // odd number
    const int NUMBER_OF_POINTS = 50;
    if (AREA_SIZE % 2 == 0) throw runtime_error("AREA_SIZE needs to be odd number");

    for (int iteration = 0; iteration < NUMBER_OF_POINTS; ++iteration) {
        pair<int, int> position = utils::getRandomPosition(imagePair.getDepth(), AREA_SIZE);

        vector<Vector3f> pointsVector;
        for (int i = position.first - (AREA_SIZE - 1) / 2; i <= position.first + (AREA_SIZE - 1) / 2; ++i) {
            for (int j = position.second - (AREA_SIZE - 1) / 2; j <= position.second + (AREA_SIZE - 1) / 2; ++j) {
                pointsVector.push_back(Vector3f(i, j, imagePair.getDepthAt(i, j)));
            }
        }
        Plane plane = PlaneRansac::getPlane(pointsVector);
        Vec3b color = plane.isValid() ? Vec3b(0, 255, 0) : Vec3b(0, 0, 255);
        for (Vector3f vector : pointsVector) {
            utils::paintPixel((Mat &) imagePair.getRgb(), vector, color);
        }
    }
    visualizer.updateCloud(imagePair.getRgb(), imagePair.getDepth());
    return application.exec();*/
    std::vector<cv::Point_<float>> pointsVec;
    std::vector<Cluster> clustersVec;
//    cv::Point_<float> pointsArray[4] = {cv::Point_<float>(0.0f, 0.0f), cv::Point_<float>(2.0f, 3.0f),
//                                        cv::Point_<float>(3.0f, 2.0f), cv::Point_<float>(3.0f, 1.0f)};

//    cv::Point_<float> pointsArray[6] = {cv::Point_<float>(0.0f, 0.0f), cv::Point_<float>(1.0f, 1.0f),
//                                        cv::Point_<float>(3.0f, 3.0f), cv::Point_<float>(6.0f, 6.0f),
//                                        cv::Point_<float>(10.0f, 10.0f), cv::Point_<float>(15.0f, 15.0f)};

    cv::Point_<float> pointsArray[7] = {cv::Point_<float>(0.0f, 0.0f), cv::Point_<float>(2.0f, 0.0f),
                                        cv::Point_<float>(4.0f, 0.0f), cv::Point_<float>(7.0f, 0.0f),
                                        cv::Point_<float>(9.0f, 0.0f), cv::Point_<float>(14.0f, 0.0f),
                                        cv::Point_<float>(16.0f, 0.0f)};

    for(int i=0; i<7; ++i){
        pointsVec.push_back(pointsArray[i]);
    }
    Clustering::computeClusters(pointsVec, clustersVec);

    for(Cluster cluster : clustersVec){
        cout << cluster.getFirstLinkIndex() << " " << cluster.getSecondLinkIndex() << " "
             << cluster.getDistanceBetweenLinks()<<std::endl;
    }

    std::vector<std::unordered_set<int>> vecEachClusterPoints;

    Clustering::getClustersAfterThreshold(2.5, pointsVec, vecEachClusterPoints);

    cout << endl <<"Number of clusters: " << vecEachClusterPoints.size() << endl << endl;

    for(std::unordered_set<int> points: vecEachClusterPoints){
        for(int index: points){
            cout<< index << " ";
        }
        cout << endl;
    }
}

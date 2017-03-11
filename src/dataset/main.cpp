//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "../../include/dataset/main.h"
#include "include/dataset/PlanePca.h"
#include <limits>

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

void getPoint(unsigned int u, unsigned int v, float depth, Vector3d &point3D) {
    Vector3d point(u, v, 1);
    point3D = depth * PHCP_MODEL * point;
    point3D(1) = point3D(1);
    point3D(2) = point3D(2);
}

int main(int argc, char **argv) {
    //startSlides();

    /*QApplication application(argc,argv);
    glutInit(&argc, argv);

    QGLVisualizer visu;
    visu.setWindowTitle("Dataset viewer");
    visu.show();

    return application.exec();*/

    /*Vector3f vec1(0, 0, 1), vec2(1, 1, 0), vec3(1, 0, 0.5);
    vector<Vector3f> pointsVector;
    pointsVector.push_back(vec1);
    pointsVector.push_back(vec2);
    pointsVector.push_back(vec3);
    Plane plane = PlanePca::getPlane(pointsVector);
    cout<<plane.getA() << " " << plane.getB() << " " << plane.getC() << " " << plane.getD() << endl;*/

/*    ImageLoader imageLoader(1);
    ImagePair imagePair = imageLoader.getNextPair();
    //imshow("rgb", imagePair.getRgb());
    //imshow("depth", imagePair.getDepth());
    Mat imgRGB = imread("../dataset_photos//rgb//49.png");//imagePair.getRgb();
    Mat imgDepth = imread("../dataset_photos//depth//49.png", CV_LOAD_IMAGE_ANYDEPTH);//imagePair.getDepth();

    cout << "cols: " << imgDepth.cols << " rows: " << imgDepth.rows << endl;
    cout << "cols: " << imgRGB.cols << " rows: " << imgRGB.rows << endl;

    QApplication application(argc,argv);
    glutInit(&argc, argv);
    QGLVisualizer visu;
    visu.setPGCPModel(PHCP_MODEL);
    visu.setWindowTitle("Dataset viewer");
    visu.updateCloud(imgRGB, imgDepth);



    for (int i = 0; i < 10; ++i) {
        int x = rand() % (imgDepth.cols - 20);
        int y = rand() % (imgDepth.rows - 20);
        //cout << "x: " << x << " y: " << y << endl;
        Rect myROI(x, y, 20, 20);
        Mat cropedImageRGB = imgRGB(myROI);
        Mat cropedImageDepth = imgDepth(myROI);
        //imshow(to_string(i), cropedImageRGB);
        Vector3d point;
        vector<Vector3f> pointsVector;
        for (unsigned int i = 0; i < cropedImageDepth.rows; i++) {
            for (unsigned int j = 0; j < cropedImageDepth.cols; j++) {
                float depthM = float(cropedImageDepth.at<uint16_t>(i, j)) /5000.0f;
                getPoint(x + j, y + i, depthM, point);
                point(1) = -point(1);
                point(2) = -point(2);
                pointsVector.push_back(point.cast<float>());
            }
            visu.addToCloud(pointsVector);
        }
        Plane plane = PlanePca::getPlane(pointsVector);
        cout<<"Plane parameters:"<<plane.getA() << " " << plane.getB() << " " << plane.getC() << " " << plane.getD() << endl;
    }

    visu.show();
    application.exec();*/

    std::vector<cv::Point_<float>> pointsVec;
    std::vector<Cluster> clusterVec;
    cv::Point_<float> tab[7] = {cv::Point_<float>(-3.0f, 3.0f),
                                cv::Point_<float>(0.0f, 0.0f),
                                cv::Point_<float>(0.0f, -3.0f),
                                cv::Point_<float>(2.0f, 3.0f),
                                cv::Point_<float>(3.0f, 2.0f),
                                cv::Point_<float>(3.0f, 1.0f),
                                cv::Point_<float>(4.0f, -3.0f)};
//    cv::Point_<float> tab[4] = {cv::Point_<float>(0.0f, 0.0f),
//                            cv::Point_<float>(2.0f, 3.0f),
//                            cv::Point_<float>(3.0f, 2.0f),
//                            cv::Point_<float>(3.0f, 1.0f),};
    for (int i = 0; i < 7; ++i) {
        pointsVec.push_back(tab[i]);
    }
    Clustering::computeClasters(pointsVec, clusterVec);

    for (int i = 0; i < clusterVec.size(); ++i) {
        std::cout<<"First index: " << clusterVec.at(i).getFirstLinkIndex() << " Second index: "
                 << clusterVec.at(i).getSecondLinkIndex() << " Distance: " << clusterVec.at(i).getDistanceBetweenLinks()<<endl;
    }


    return -1;
}

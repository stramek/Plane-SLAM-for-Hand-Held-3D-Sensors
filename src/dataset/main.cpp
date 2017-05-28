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



    while (true) {
        ImageLoader imageLoader(50);

        vector<Plane> planeVectorPreviousFrame;
        vector<Plane> planeVectorCurrentFrame;
        vector<pair<Plane, Plane>> similarPlanes;

        const int AREA_SIZE = 31;
        const int NUMBER_OF_POINTS = 200;

        cout<<endl<<endl<<endl<<"Filling vectors with planes..."<<endl;
        ImagePair imagePair1 = imageLoader.getNextPair();

        make_unique<PlaneFillerBuilder>()
                ->withDataset(&imagePair1)
                //->withPlaneDetector(new PlaneDetector())
                ->withAreaSize(AREA_SIZE)
                ->withNumberOfPoints(NUMBER_OF_POINTS)
                ->build()
                ->fillVector(&planeVectorPreviousFrame);

        ImagePair imagePair2 = imageLoader.getNextPair(20);

        make_unique<PlaneFillerBuilder>()
                ->withDataset(&imagePair2)
                ->withAreaSize(AREA_SIZE)
                ->withNumberOfPoints(NUMBER_OF_POINTS)
                ->withPreviousPlanePercent(&planeVectorPreviousFrame, 0.5)
                ->build()
                ->fillVector(&planeVectorCurrentFrame);

        //planeUtils::displayClusteredPlanes(imagePair1, planeVectorPreviousFrame);
        //planeUtils::displayClusteredPlanes(imagePair2, planeVectorCurrentFrame);

        cout<<"Merging planes..."<<endl;
        planeUtils::mergePlanes(planeVectorPreviousFrame);
        planeUtils::mergePlanes(planeVectorCurrentFrame);
//
        cout<<"Finding similar planes..."<<endl;
        similarPlanes = planeUtils::getSimilarPlanes(planeVectorPreviousFrame, planeVectorCurrentFrame);

        cout<<"Filtering pairs by angle..."<<endl;
        cout<<"Finished!"<<endl;
        planeUtils::visualizeSimilarPlanes(similarPlanes, imagePair1.getRgb(), imagePair2.getRgb());
        //visualizer.updateCloud(imagePair2.getRgb(), imagePair2.getDepth());
        //visualizer.updatePlanes(planeVectorCurrentFrame);

        waitKey();

        //utils::generateOctoMap("Dataset", visualizer.getPointCloud());
    }


    return application.exec();
}

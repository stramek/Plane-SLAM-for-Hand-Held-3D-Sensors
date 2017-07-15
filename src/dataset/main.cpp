//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "include/dataset/main.h"

vector<Plane> planeVectorPreviousFrame;
vector<Plane> planeVectorCurrentFrame;
vector<pair<Plane, Plane>> similarPlanes;

double sTod(string s) {
    return boost::lexical_cast<double>(s);
}

void loadDatasetLines(vector<PosOrient>& positions) {

    cout<<"Loading file \""<<"dataset_data.txt"<<"\"... ";
    ifstream myfile("dataset_data.txt");

    if (myfile.good()) {
        std::string line;
        while (std::getline(myfile, line)) {
            std::vector<std::string> loadedValues;
            boost::split(loadedValues, line, boost::is_any_of(" "));

            PosOrient posOrient(Vector3d(sTod(loadedValues[1]), sTod(loadedValues[2]), sTod(loadedValues[3])),
                                Vector4d(sTod(loadedValues[4]), sTod(loadedValues[5]), sTod(loadedValues[6]), sTod(loadedValues[7])));
            positions.push_back(posOrient);
        }
        cout<<"Success!"<<endl;
    } else {
        cout<<"Could not load file!"<<endl;
        exit(-1);
    }
}

int main(int argc, char **argv) {
    QApplication application(argc, argv);
    glutInit(&argc, argv);

//    QGLVisualizer visualizer;
//    visualizer.setWindowTitle("Dataset viewer");
//    visualizer.setPHCPModel(PHCP_MODEL);
//    visualizer.show();

    vector<PosOrient> positions;
    loadDatasetLines(positions);

    const int AREA_SIZE = 35;
    const int NUMBER_OF_POINTS = 300;

    ImageLoader imageLoader(50);
    ImagePair currentFrame;

    for (int i = 0; i < imageLoader.getNumberOfPhotos(); ++i) {
        currentFrame = imageLoader.getNextPair();

        make_unique<PlaneFillerBuilder>()
                ->withDataset(&currentFrame)
                ->withPlaneDetector(new PcaPlaneDetector())
                ->withAreaSize(AREA_SIZE)
                ->withNumberOfPoints(NUMBER_OF_POINTS)
                ->withPreviousPlanePercent(&planeVectorPreviousFrame, 0.5)
                ->build()
                ->fillVector(&planeVectorCurrentFrame);

        planeUtils::mergePlanes(planeVectorCurrentFrame);

        if (!planeVectorPreviousFrame.empty()) {
            similarPlanes = planeUtils::getSimilarPlanes(planeVectorPreviousFrame, planeVectorCurrentFrame);
            cout<<"Frame "<<i<<"-"<<i+1<<" found: "<<similarPlanes.size()<<" similar planes."<<endl;
        }

        planeVectorPreviousFrame.clear();
        copy(planeVectorCurrentFrame.begin(), planeVectorCurrentFrame.end(), back_inserter(planeVectorPreviousFrame));
        planeVectorCurrentFrame.clear();
    }








    /*while (true) {
        ImageLoader imageLoader(50);

        const int AREA_SIZE = 35;
        const int NUMBER_OF_POINTS = 300;

        cout<<endl<<endl<<endl<<"Filling vectors with planes..."<<endl;
        ImagePair imagePair1 = imageLoader.getNextPair();

        make_unique<PlaneFillerBuilder>()
                ->withDataset(&imagePair1)
                ->withPlaneDetector(new PcaPlaneDetector())
                ->withAreaSize(AREA_SIZE)
                ->withNumberOfPoints(NUMBER_OF_POINTS)
                ->build()
                ->fillVector(&planeVectorPreviousFrame);

        ImagePair imagePair2 = imageLoader.getNextPair(40);

        make_unique<PlaneFillerBuilder>()
                ->withDataset(&imagePair2)
                ->withPlaneDetector(new PcaPlaneDetector())
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
    }*/


    return application.exec();
}

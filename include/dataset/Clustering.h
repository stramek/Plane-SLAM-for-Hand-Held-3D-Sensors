//
// Created by mordimer on 11.03.17.
//

#ifndef PROJEKTMAGISTERSKI_CLUSTERING_H
#define PROJEKTMAGISTERSKI_CLUSTERING_H

#include <istream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "Cluster.h"

class Clustering {
private:
    class SimilarityMatrixItem{
    public:
        float similarity;
        int index;
    };
    static float getDistanceBetweenTwoPoints(cv::Point_<float> point1, cv::Point_<float> point2);

public:
    static void computeClasters(std::vector<cv::Point_<float>> pointsVec, std::vector<Cluster> &clastersVec);
};


#endif //PROJEKTMAGISTERSKI_CLUSTERING_H

//
// Created by mordimer on 11.03.17.
//

#ifndef PROJEKTMAGISTERSKI_CLUSTERING_H
#define PROJEKTMAGISTERSKI_CLUSTERING_H

#include <istream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "Cluster.h"
#include <unordered_set>

class Clustering {
private:
    class SimilarityItem{
    public:
        float similarity;
        int index;
    };
    static float getDistanceBetweenTwoPoints(cv::Point_<float> point1, cv::Point_<float> point2);

    static void createSimilarityMatrix(SimilarityItem **&similarityMatrix, unsigned long size);
    static void deleteSimilarityMatrix(SimilarityItem **&similarityMatrix, unsigned long size);
    static void createNextBestMergeMatrix(SimilarityItem *&nextBestMerge, unsigned long size);
    static void deleteNextBestMergeMatrix(SimilarityItem *&nextBestMerge);
    static void clusteringInitializeStep(SimilarityItem **&similarityMatrix, SimilarityItem *&nextBestMerge,
                                         int *&I, const std::vector<cv::Point_<float>> &pointsVec);
public:
    static void computeClusters(std::vector<cv::Point_<float>> pointsVec, std::vector<Cluster> &clustersVec);

    static void getClustersAfterThreshold(float cutThreshold, std::vector<cv::Point_<float>> pointsVec,
                                          std::vector<std::unordered_set<int>> &vecEachClusterPoints);
};


#endif //PROJEKTMAGISTERSKI_CLUSTERING_H

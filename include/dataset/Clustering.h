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
#include "../models/Plane.h"

class Clustering {
private:

    const static float MAX_ANGLE_THRESHOLD;

    class SimilarityItem{
    public:
        float similarity;
        unsigned int index;
    };
    static float getDistanceBetweenTwoPoints(cv::Point_<float> point1, cv::Point_<float> point2);
    static float getDistanceBetweenTwoPlanes(const Plane &firstPlane, const Plane &secondPlane);
    static float getAngleBetweenTwoPlanes(const Plane &firstPlane, const Plane &secondPlane);
    static float getSimilarityOfTwoPlanes(const Plane &firstPlane, const Plane &secondPlane);

    static void createSimilarityMatrix(SimilarityItem **&similarityMatrix, unsigned long size);
    static void deleteSimilarityMatrix(SimilarityItem **&similarityMatrix, unsigned long size);
    static void createNextBestMergeMatrix(SimilarityItem *&nextBestMerge, unsigned long size);
    static void deleteNextBestMergeMatrix(SimilarityItem *&nextBestMerge);
    static void clusteringInitializeStep(SimilarityItem **&similarityMatrix, SimilarityItem *&nextBestMerge,
                                         unsigned int *&I, const std::vector<Plane> &planesVec);
    static void computeIndexOfTwoPlanesToMerge(SimilarityItem *&nextBestMerge, unsigned int *&I,
                                               unsigned int &firstPlaneIndexToMerge,
                                               unsigned int &secondPlaneIndexToMerge, unsigned long size);
    static void addNewClusterToVec(std::vector<Cluster> &clustersVec, SimilarityItem **&similarityMatrix,
                                   const unsigned int &firstPlaneIndexToMerge,
                                   const unsigned int &secondPlaneIndexToMerge);
    static void updateSimilarityMatrixState(SimilarityItem **&similarityMatrix, unsigned int *&I,
                                            const unsigned int &firstPlaneIndexToMerge,
                                            const unsigned int &secondPlaneIndexToMerge, unsigned long size);
    static void updateNextBestMerge(SimilarityItem **&similarityMatrix ,SimilarityItem *&nextBestMerge, unsigned int *&I,
                                    unsigned int firstPlaneIndexToMerge,unsigned long size);

    static float getDistanceBetweenPointAndPlane(Plane plane, Vector3f point);

public:
    static void computeClusters(std::vector<Plane> planesVec, std::vector<Cluster> &clustersVec);

    static void getClustersAfterThreshold(float cutThreshold, std::vector<Plane> planesVec,
                                          std::vector<std::unordered_set<int>> &vecEachClusterPlanes);
    static void getClusteredPlaneGroup(std::vector<Plane> planesVec, vector<vector<Plane>>& clusteredPlanes);
    static vector<Plane> getAveragedPlanes(vector<vector<Plane>>& clusteredPlanes);
};


#endif //PROJEKTMAGISTERSKI_CLUSTERING_H

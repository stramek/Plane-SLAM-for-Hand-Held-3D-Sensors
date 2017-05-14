//
// Created by mordimer on 11.03.17.
//

#ifndef PROJEKTMAGISTERSKI_CLUSTERING_H
#define PROJEKTMAGISTERSKI_CLUSTERING_H

#include <istream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "include/models/Cluster.h"
#include <unordered_set>
#include "include/models/Plane.h"

class Clustering {
private:

    std::priority_queue<Cluster> priorityQueueDistance;
    float cutSimilarity = 0;

    const static float MAX_ANGLE_THRESHOLD;

    static float getDistanceBetweenTwoPoints(cv::Point_<float> point1, cv::Point_<float> point2);
    static float getDistanceBetweenTwoPlanes(const Plane &firstPlane, const Plane &secondPlane);
    static float getAngleBetweenTwoPlanes(const Plane &firstPlane, const Plane &secondPlane);
    static float getSimilarityOfTwoPlanes(const Plane &firstPlane, const Plane &secondPlane);

    static float getDistanceBetweenPointAndPlane(Plane plane, Vector3f point);


    double findMinDistance(std::pair<int,int>& pairedIds);
    void findPartsInClusters(const std::vector<std::vector<int>>& clusters, const std::pair<int,int>& pairedIds, std::pair<int,int>& clustersIds) const;
    void computeDistanceMatrix(const std::vector<Plane> &planesVec, std::vector<std::vector<double>>& distanceMatrix);
    bool mergeTwoClusters(std::vector<std::vector<int>>& clusters, const std::pair<int,int>& clustersIds, const std::vector<std::vector<double>>& distanceMatrix) const;
    double computeMaxDist(const std::vector<std::vector<int>>& clusters, const std::pair<int,int>& clustersIds, const std::vector<std::vector<double>>& distanceMatrix) const;
    void getClusteredPlaneGroup(const std::vector<std::vector<int>> clusters, const std::vector<Plane> &planesVec, vector<vector<Plane>> &clusteredPlanes);

public:
    void selectParts(const std::vector<Plane> &planesVec, std::vector<std::vector<Plane>> &clusteredPlanes);

    static vector<Plane> getAveragedPlanes(vector<vector<Plane>>& clusteredPlanes);

    void setCutSimilarity(float cutSimilarity);
};


#endif //PROJEKTMAGISTERSKI_CLUSTERING_H

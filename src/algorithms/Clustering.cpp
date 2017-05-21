//
// Created by mordimer on 11.03.17.
//

#include "include/algorithms/Clustering.h"

const double Clustering::MAX_ANGLE_THRESHOLD = 5.0;

double Clustering::getDistanceBetweenPointAndPlane(Plane plane, Vector3d point) {
    double distance = abs(plane.getA() * point(0) + plane.getB() * point(1) + plane.getC() * point(2) + plane.getD()) /
                     sqrtf(powf(plane.getA(), 2) + powf(plane.getB(), 2) + powf(plane.getC(), 2));
    return distance;
}

double Clustering::getDistanceBetweenTwoPlanes(const Plane &firstPlane, const Plane &secondPlane) {
/*    double distance = 0;
    vector<Vector3d> pointsVec = secondPlane.getPoints();

    for (int i = 0; i < 50; ++i) {
        random_device rd;
        mt19937 rng(rd());
        uniform_int_distribution<unsigned int> pointIndex(0, secondPlane.getNumberOfPoints() - 1);
        Vector3d randomPointOnSecondPlane = pointsVec[pointIndex(rng)];
        distance += getDistanceBetweenPointAndPlane(firstPlane, randomPointOnSecondPlane);
    }

    return distance / 50;*/
    Vector3d firstVec(firstPlane.getD()/firstPlane.getA(), firstPlane.getD()/firstPlane.getB(), firstPlane.getD()/firstPlane.getC());
    Vector3d secondVec(secondPlane.getD()/secondPlane.getA(), secondPlane.getD()/secondPlane.getB(), secondPlane.getD()/secondPlane.getC());

    Vector3d diffVec = firstVec - secondVec;

    return sqrtf(powf(diffVec(0), 2.0) + powf(diffVec(1), 2.0) + powf(diffVec(2), 2.0));
}

double Clustering::getAngleBetweenTwoPlanes(const Plane &firstPlane, const Plane &secondPlane) {
    Eigen::Vector3d firstPlaneNormalVec = firstPlane.getPlaneNormalVec();
    Eigen::Vector3d secondPlaneNormalVec = secondPlane.getPlaneNormalVec();

    double angleCos =
            firstPlaneNormalVec.dot(secondPlaneNormalVec) / firstPlaneNormalVec.norm() / secondPlaneNormalVec.norm();
    if (angleCos < -1) angleCos = -1.0f;
    if (angleCos > 1) angleCos = 1.0f;
    double angle = acosf(angleCos) * 180.0f / (double) M_PI;
    if (angle > 90.0f) {
        angle = 180.0f - angle;
    }
    return angle;
}

double Clustering::getSimilarityOfTwoPlanes(const Plane &firstPlane, const Plane &secondPlane) {
    double angleBetweenTwoPlanes = firstPlane.getAngleBetweenTwoPlanes(secondPlane);
    return  angleBetweenTwoPlanes;
    //if(abs(angleBetweenTwoPlanes) < MAX_ANGLE_THRESHOLD){
        //return getDistanceBetweenTwoPlanes(firstPlane, secondPlane);
//        std::cout<<"First plane:" << firstPlane.getA() << " " << firstPlane.getB() << " " << firstPlane.getC()
//                 << " " << firstPlane.getD() << " vec length: " << sqrtf(powf(firstPlane.getA(), 2.0f) + powf(firstPlane.getB(), 2.0f) + powf(firstPlane.getC(), 2.0f));
//        std::cout<<" Second plane:" << secondPlane.getA() << " " << secondPlane.getB() << " " << secondPlane.getC()
//                 << " " << secondPlane.getD() << " vec length: " << sqrtf(powf(secondPlane.getA(), 2.0f) + powf(secondPlane.getB(), 2.0f) + powf(secondPlane.getC(), 2.0f)) << std::endl;
    //    return abs(firstPlane.getD() - secondPlane.getD());
   // }
    //return std::numeric_limits<float>::max() / 2.0f;
    //return abs(angleBetweenTwoPlanes);
}

vector<Plane> Clustering::getAveragedPlanes(vector<vector<Plane>> &clusteredPlanes) {
    vector<Plane> averagedPlanesVec;
    for (auto planesFromCluster : clusteredPlanes) {
        double averagedD = 0;
        int averagedHue = 0;
        int averagedSaturation = 0;
        int averagedValue = 0;
        vector<Vector3d> mergedPlanePoints;
        vector<ImageCoords> mergedPlaneImageCoordsVec;
        Vector3d averagedNormalVec = Vector3d::Zero();
        for (auto plane : planesFromCluster) {
            averagedNormalVec += plane.getPlaneNormalVec();
            averagedD += plane.getD();
            averagedHue += plane.getColor().getHue();
            averagedSaturation += plane.getColor().getSaturation();
            averagedValue += plane.getColor().getValue();

            vector<Vector3d> points = plane.getPoints();
            vector<ImageCoords> imageCoordsVec = plane.getImageCoordsVec();
            mergedPlanePoints.insert(mergedPlanePoints.end(), points.begin(), points.end());
            mergedPlaneImageCoordsVec.insert(mergedPlaneImageCoordsVec.end(), imageCoordsVec.begin(),
                                             imageCoordsVec.end());
        }
        averagedNormalVec = averagedNormalVec / planesFromCluster.size();
        averagedD = averagedD / planesFromCluster.size();
        averagedHue = averagedHue / (int) planesFromCluster.size();
        averagedSaturation = averagedSaturation / (int) planesFromCluster.size();
        averagedValue = averagedValue / (int) planesFromCluster.size();
        Plane averagedPlane(averagedNormalVec, averagedD, mergedPlanePoints, mergedPlaneImageCoordsVec,
                            HSVColor((uint8_t) averagedHue, (uint8_t) averagedSaturation, (uint8_t) averagedValue));
        averagedPlanesVec.push_back(averagedPlane);
    }
    return averagedPlanesVec;
}


////////////////////////////

void Clustering::selectParts(const std::vector<Plane> &planesVec, std::vector<std::vector<Plane>> &clusteredPlanes){
    std::vector<std::vector<double>> distanceMatrix(planesVec.size(), std::vector<double>(planesVec.size()));
    //computeDistanceMatrix(dictionary, hierarchy, distanceMatrix, transformMatrix);
    computeDistanceMatrix(planesVec, distanceMatrix);

    if(DEBUG){
        std::cout << std::setprecision(1) << std::fixed;
        for(auto row : distanceMatrix){
            for(auto element : row){
                std::cout<<element << "  ";
            }
            std::cout<<std::endl;
        }

        std::cout<<std::endl;
    }

    std::vector<std::vector<int>> clusters(planesVec.size());
    for (size_t i=0;i<clusters.size();i++){
        clusters[i].push_back((int)i);
    }

    for (size_t i=0;i<(planesVec.size()-1)*planesVec.size()/2;i++){

        std::pair<int,int> pairedIds;
        double minDist = findMinDistance(pairedIds);
        if(DEBUG){
            std::cout<<"PairedIds with min distance: " << pairedIds.first << " " << pairedIds.second << " distance: " << minDist <<std::endl;
        }

        if (minDist>=cutSimilarity)
            break;

        //merge two centroids
        std::pair<int,int> clustersIds;
        findPartsInClusters(clusters, pairedIds, clustersIds);
        if(DEBUG){
            std::cout<<"ClustersIds: " << clustersIds.first << " " << clustersIds.second <<std::endl;
        }

        if (clustersIds.first!=clustersIds.second){
            mergeTwoClusters(clusters, clustersIds, distanceMatrix);
        }
    }

    if(DEBUG){
        for(auto singleCluster : clusters){
            for(auto planeIndex : singleCluster){
                std::cout<< planeIndex << " ";
            }
            std::cout<< std::endl;
        }
        std::cout<< std::endl;
        std::cout<< std::endl;
    }

    getClusteredPlaneGroup(clusters, planesVec, clusteredPlanes);

    if(DEBUG){
        for(auto row : distanceMatrix){
            for(auto element : row){
                std::cout<<element << "  ";
            }
            std::cout<<std::endl;
        }

        std::cout<<std::endl;
    }
}

void Clustering::computeDistanceMatrix(const std::vector<Plane> &planesVec, std::vector<std::vector<double>>& distanceMatrix){
    while( !priorityQueueDistance.empty() ) priorityQueueDistance.pop();
    size_t startId(0), endId(planesVec.size());
    for (size_t idA=startId;idA<endId;idA++){
        for (size_t idB=idA;idB<endId;idB++){
            double dist(0);
            if (idA==idB){
                distanceMatrix[idB][idA]=0;
            }
            else{

                dist = getSimilarityOfTwoPlanes(planesVec.at(idA), planesVec.at(idB));
                distanceMatrix[idA][idB]=dist;
                distanceMatrix[idB][idA]=dist;
                Cluster cluster;
                cluster.setDistanceBetweenLinks(dist);
                cluster.setLinkedIndexes(std::make_pair(idA,idB));
                priorityQueueDistance.push(cluster);
            }
        }
    }
}

/// find min distance int the distance matrix
double Clustering::findMinDistance(std::pair<int,int>& pairedIds){
    Cluster cluster = priorityQueueDistance.top();
    double minDist = cluster.getDistanceBetweenLinks();
    pairedIds = cluster.getLinkedIndexes();
    priorityQueueDistance.pop();

    return minDist;
}

void Clustering::setCutSimilarity(double cutSimilarity){
    Clustering::cutSimilarity = cutSimilarity;
}

/// find clusters ids to which contain specyfic parts (pairedIds)
void Clustering::findPartsInClusters(const std::vector<std::vector<int>>& clusters, const std::pair<int,int>& pairedIds, std::pair<int,int>& clustersIds) const{
    bool found[2]={false, false};
    for (size_t i=0;i<clusters.size();i++){
        for (auto &id : clusters[i]){
            if (id==pairedIds.first){
                clustersIds.first = (int)i;
                found[0]=true;
            }
            if (id==pairedIds.second){
                clustersIds.second = (int)i;
                found[1]=true;
            }
            if (found[0]&&found[1]) break;
        }
        if (found[0]&&found[1]) break;
    }
}

/// merge two clusters
bool Clustering::mergeTwoClusters(std::vector<std::vector<int>>& clusters, const std::pair<int,int>& clustersIds, const std::vector<std::vector<double>>& distanceMatrix) const{
    double maxDist = computeMaxDist(clusters, clustersIds, distanceMatrix);
    if (maxDist<cutSimilarity){
        if (clustersIds.first<clustersIds.second){
            // merge clusters
            clusters[clustersIds.first].insert(clusters[clustersIds.first].end(), clusters[clustersIds.second].begin(), clusters[clustersIds.second].end());
            // remove second cluster
            clusters.erase(clusters.begin()+clustersIds.second);
        }
        else{
            // merge clusters
            clusters[clustersIds.second].insert(clusters[clustersIds.second].end(), clusters[clustersIds.first].begin(), clusters[clustersIds.first].end());
            // remove second cluster
            clusters.erase(clusters.begin()+clustersIds.first);
        }
        return true;
    }
    else
        return false;
}

/// compute max distance between centroid of the first cluster and all parts in the second cluster
double Clustering::computeMaxDist(const std::vector<std::vector<int>>& clusters, const std::pair<int,int>& clustersIds, const std::vector<std::vector<double>>& distanceMatrix) const{
    int partIdA = clustersIds.first;
    double maxDist = std::numeric_limits<double>::min();
/*    for (const auto &partIdB : clusters[clustersIds.second]){
        double dist;
        if (partIdA>partIdB)//because up-triangle elements in distance matrix are cleaned
            dist = distanceMatrix[partIdA][partIdB];
        else
            dist = distanceMatrix[partIdB][partIdA];
        if (dist>maxDist){
            maxDist = dist;
        }
    }*/

    for (const auto &partIdA : clusters[clustersIds.first]){
        for (const auto &partIdB : clusters[clustersIds.second]){
            double dist;
            if (partIdA>partIdB)//because up-triangle elements in distance matrix are cleaned
                dist = distanceMatrix[partIdA][partIdB];
            else
                dist = distanceMatrix[partIdB][partIdA];
            if (dist>maxDist){
                maxDist = dist;
            }
        }
    }
    return maxDist;
}

void Clustering::getClusteredPlaneGroup(const std::vector<std::vector<int>> clusters, const std::vector<Plane> &planesVec, vector<vector<Plane>> &clusteredPlanes) {
    for (auto planesIndexesInOneCluster : clusters) {
        vector<Plane> singleCluster;
        for (auto &index : planesIndexesInOneCluster) {
            Plane singleClusterPlane = planesVec.at(index);
            singleCluster.push_back(singleClusterPlane);
        }
        clusteredPlanes.push_back(singleCluster);
    }
}
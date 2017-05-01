//
// Created by mordimer on 11.03.17.
//

#include "include/dataset/Clustering.h"


void Clustering::computeClusters(std::vector<cv::Point_<float>> pointsVec, std::vector<Cluster> &clustersVec) {
    SimilarityMatrixItem **SimilarityMatrix;

    SimilarityMatrix = new SimilarityMatrixItem *[pointsVec.size()];
    for (int i = 0; i < pointsVec.size(); ++i) {
        SimilarityMatrix[i] = new SimilarityMatrixItem[pointsVec.size()];
    }

    int *I = new int[pointsVec.size()];
    SimilarityMatrixItem *NBM = new SimilarityMatrixItem[pointsVec.size()];

    for (int i = 0; i < pointsVec.size(); ++i) {
        int indexMaxRow;
        float maxSimilarityRow = std::numeric_limits<float>::max();
        for (int j = 0; j < pointsVec.size(); ++j) {
            SimilarityMatrix[i][j].similarity = getDistanceBetweenTwoPoints(pointsVec.at(i), pointsVec.at(j));
            SimilarityMatrix[i][j].index = j;
            if (maxSimilarityRow > SimilarityMatrix[i][j].similarity && i != j) {
                maxSimilarityRow = SimilarityMatrix[i][j].similarity;
                indexMaxRow = j;
            }
        }
        I[i] = i;
        NBM[i] = SimilarityMatrix[i][indexMaxRow];
    }

    for (int i = 0; i < pointsVec.size() - 1; ++i) {
        int i1, i2;
        float maxNBM_Sim = std::numeric_limits<float>::max();

        for (int j = 0; j < pointsVec.size(); ++j) {
            if (NBM[j].similarity < maxNBM_Sim && I[j] == j) {
                maxNBM_Sim = NBM[j].similarity;
                i1 = j;
            }
        }

        i2 = I[NBM[i1].index];
        Cluster cluster;
        cluster.setFirstLinkIndex(i1);
        cluster.setSecondLinkIndex(i2);
        cluster.setDistanceBetweenLinks(SimilarityMatrix[i1][i2].similarity);
        clustersVec.push_back(cluster);

        for (int j = 0; j < pointsVec.size(); ++j) {
            if (I[j] == j && j != i1 && j != i2) {
                if (SimilarityMatrix[i1][j].similarity < SimilarityMatrix[i2][j].similarity) {
                    SimilarityMatrix[i1][j].similarity = SimilarityMatrix[i1][j].similarity;
                    SimilarityMatrix[j][i1].similarity = SimilarityMatrix[i1][j].similarity;
                } else {
                    SimilarityMatrix[i1][j].similarity = SimilarityMatrix[i2][j].similarity;
                    SimilarityMatrix[j][i1].similarity = SimilarityMatrix[i2][j].similarity;
                }
            }
            if (I[j] == i2) {
                I[j] = i1;
            }
        }

        SimilarityMatrixItem similarityMatrixItem_MaxSim;
        similarityMatrixItem_MaxSim.similarity = std::numeric_limits<float>::max();;
        for (int j = 0; j < pointsVec.size(); ++j) {
            if (SimilarityMatrix[i1][j].similarity < similarityMatrixItem_MaxSim.similarity && I[j] == j && j != i1) {
                similarityMatrixItem_MaxSim = SimilarityMatrix[i1][j];
            }
        }
        NBM[i1] = similarityMatrixItem_MaxSim;
    }
}

float Clustering::getDistanceBetweenTwoPoints(cv::Point_<float> point1, cv::Point_<float> point2) {
    return sqrtf(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

void Clustering::getClustersAfterThreshold(float cutThreshold, std::vector<cv::Point_<float>> pointsVec,
                                           std::vector<std::unordered_set<int>> &vecEachClusterPoints) {
    std::vector<Cluster> clustersVec;
    computeClusters(pointsVec, clustersVec);

    std::unordered_set<int> *point = NULL;
    int clusterNumber = 0;
    for (Cluster cluster : clustersVec) {
        if (cluster.getDistanceBetweenLinks() > cutThreshold) {
            for (int i = 0; i < pointsVec.size(); ++i) {
                bool isInCluster = false;
                for (std::unordered_set<int> unordered_setPoints : vecEachClusterPoints) {
                    if (unordered_setPoints.find(i) !=
                        unordered_setPoints.end()) {
                        isInCluster = true;
                    }
                }
                if(!isInCluster){
                    point = new std::unordered_set<int>;
                    point->insert(i);
                    vecEachClusterPoints.push_back(*point);
                }
            }
        } else {
            if (
                    vecEachClusterPoints.size() == 0 &&
                    point == NULL) {
                point = new std::unordered_set<int>;
                point->insert(cluster.getFirstLinkIndex());
                point->insert(cluster.getSecondLinkIndex());
                vecEachClusterPoints.push_back(*point);

            } else if (
                    vecEachClusterPoints.at(clusterNumber).find(cluster.getFirstLinkIndex()) ==
                    vecEachClusterPoints.at(clusterNumber).end() &&
                    vecEachClusterPoints.at(clusterNumber).find(cluster.getSecondLinkIndex()) ==
                    vecEachClusterPoints.at(clusterNumber).end()) {
//                if (point != NULL) {
//                    vecEachClusterPoints.push_back(*point);
//                }
                point = new std::unordered_set<int>;
                clusterNumber++;
                point->insert(cluster.getFirstLinkIndex());
                point->insert(cluster.getSecondLinkIndex());
                vecEachClusterPoints.push_back(*point);
            } else {
                if (point != NULL) {
                    vecEachClusterPoints.at(clusterNumber).insert(cluster.getFirstLinkIndex());
                    vecEachClusterPoints.at(clusterNumber).insert(cluster.getSecondLinkIndex());
                }
            }
        }
    }

    for(int i=0;i<vecEachClusterPoints.size()-1;++i){
        bool clusterDeleteFlag = false;
        for(auto point : vecEachClusterPoints.at(i)){
            for(int j=i+1;j<vecEachClusterPoints.size();++j) {
                if(vecEachClusterPoints.at(j).find(point) != vecEachClusterPoints.at(j).end()){
                    clusterDeleteFlag = true;
                }
                if(clusterDeleteFlag) {
                    for(auto point : vecEachClusterPoints.at(j)){
                        vecEachClusterPoints.at(i).insert(point);
                    }
                    vecEachClusterPoints.erase(vecEachClusterPoints.begin() + j);
                    --j;
                }
                clusterDeleteFlag = false;
            }
        }
    }
}

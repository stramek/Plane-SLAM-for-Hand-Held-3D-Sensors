//
// Created by mordimer on 11.03.17.
//

#include "include/dataset/Clustering.h"

void Clustering::createSimilarityMatrix(SimilarityItem **&similarityMatrix, unsigned long size){
    similarityMatrix = new SimilarityItem *[size];
    for (int i = 0; i < size; ++i) {
        similarityMatrix[i] = new SimilarityItem[size];
    }
}

void Clustering::deleteSimilarityMatrix(SimilarityItem **&similarityMatrix, unsigned long size){
    for(int i = 0; i < size; ++i){
        delete[] similarityMatrix[i];
    }
    delete[] similarityMatrix;
}

void Clustering::createNextBestMergeMatrix(SimilarityItem *&nextBestMerge, unsigned long size){
    nextBestMerge = new SimilarityItem[size];
}

void Clustering::deleteNextBestMergeMatrix(SimilarityItem *&nextBestMerge){
    delete[] nextBestMerge;
}

void Clustering::clusteringInitializeStep(SimilarityItem **&similarityMatrix, SimilarityItem *&nextBestMerge,
                                          unsigned int *&I, const std::vector<cv::Point_<float>> &pointsVec){
    for (unsigned int i = 0; i < pointsVec.size(); ++i) {
        unsigned int indexMaxRow = 0;
        float maxSimilarityRow = std::numeric_limits<float>::max();
        for (unsigned int j = 0; j < pointsVec.size(); ++j) {
            similarityMatrix[i][j].similarity = getDistanceBetweenTwoPoints(pointsVec.at(i), pointsVec.at(j));
            similarityMatrix[i][j].index = j;
            if (maxSimilarityRow > similarityMatrix[i][j].similarity && i != j) {
                maxSimilarityRow = similarityMatrix[i][j].similarity;
                indexMaxRow = j;
            }
        }
        I[i] = i;
        nextBestMerge[i] = similarityMatrix[i][indexMaxRow];
    }
}

void Clustering::computeIndexOfTwoPointsToMerge(SimilarityItem *&nextBestMerge, unsigned int *&I,
                                                unsigned int &firstPointToMergeIndex,
                                                unsigned int &secondPointToMergeIndex, int size){
    float maxNBM_Sim = std::numeric_limits<float>::max();

    for (unsigned int j = 0; j < size; ++j) {
        if (nextBestMerge[j].similarity < maxNBM_Sim && I[j] == j) {
            maxNBM_Sim = nextBestMerge[j].similarity;
            firstPointToMergeIndex = j;
        }
    }

    secondPointToMergeIndex = I[nextBestMerge[firstPointToMergeIndex].index];
}

void Clustering::addNewClustertToVec(std::vector<Cluster> &clustersVec, SimilarityItem **&similarityMatrix,
                                     const unsigned int &firstPointToMergeIndex, const unsigned int &secondPointToMergeIndex){
    Cluster cluster;
    cluster.setFirstLinkIndex(firstPointToMergeIndex);
    cluster.setSecondLinkIndex(secondPointToMergeIndex);
    cluster.setDistanceBetweenLinks(similarityMatrix[firstPointToMergeIndex][secondPointToMergeIndex].similarity);
    clustersVec.push_back(cluster);
}

void Clustering::computeClusters(std::vector<cv::Point_<float>> pointsVec, std::vector<Cluster> &clustersVec) {


    unsigned const long  similarityMatSize = pointsVec.size();
    SimilarityItem **SimilarityMatrix;

    createSimilarityMatrix(SimilarityMatrix, similarityMatSize);

    unsigned int *I = new unsigned int[pointsVec.size()];
    SimilarityItem *nextBestMerge;
    createNextBestMergeMatrix(nextBestMerge, pointsVec.size());

    clusteringInitializeStep(SimilarityMatrix, nextBestMerge, I, pointsVec);

    for (int i = 0; i < pointsVec.size() - 1; ++i) {

        unsigned int firstPointToMergeIndex, secondPointToMergeIndex;
        computeIndexOfTwoPointsToMerge(nextBestMerge, I, firstPointToMergeIndex, secondPointToMergeIndex,
                                       pointsVec.size());
        addNewClustertToVec(clustersVec, SimilarityMatrix, firstPointToMergeIndex, secondPointToMergeIndex);


        for (int j = 0; j < pointsVec.size(); ++j) {
            if (I[j] == j && j != firstPointToMergeIndex && j != secondPointToMergeIndex) {
                if (SimilarityMatrix[firstPointToMergeIndex][j].similarity < SimilarityMatrix[secondPointToMergeIndex][j].similarity) {
                    SimilarityMatrix[firstPointToMergeIndex][j].similarity = SimilarityMatrix[firstPointToMergeIndex][j].similarity;
                    SimilarityMatrix[j][firstPointToMergeIndex].similarity = SimilarityMatrix[firstPointToMergeIndex][j].similarity;
                } else {
                    SimilarityMatrix[firstPointToMergeIndex][j].similarity = SimilarityMatrix[secondPointToMergeIndex][j].similarity;
                    SimilarityMatrix[j][firstPointToMergeIndex].similarity = SimilarityMatrix[secondPointToMergeIndex][j].similarity;
                }
            }
            if (I[j] == secondPointToMergeIndex) {
                I[j] = firstPointToMergeIndex;
            }
        }

        SimilarityItem similarityMatrixItem_MaxSim;
        similarityMatrixItem_MaxSim.similarity = std::numeric_limits<float>::max();;
        for (int j = 0; j < pointsVec.size(); ++j) {
            if (SimilarityMatrix[firstPointToMergeIndex][j].similarity < similarityMatrixItem_MaxSim.similarity && I[j] == j && j != firstPointToMergeIndex) {
                similarityMatrixItem_MaxSim = SimilarityMatrix[firstPointToMergeIndex][j];
            }
        }
        nextBestMerge[firstPointToMergeIndex] = similarityMatrixItem_MaxSim;
    }
    deleteSimilarityMatrix(SimilarityMatrix, similarityMatSize);
    deleteNextBestMergeMatrix(nextBestMerge);
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

    bool flag;
    do{
        flag = false;
        bool *deleteElement = new bool[vecEachClusterPoints.size()];
        for(int i =0; i<vecEachClusterPoints.size();++i){
            deleteElement[i] = false;
        }

        for(int i=0;i<vecEachClusterPoints.size()-1;++i){
            bool clusterDeleteFlag = false;
            for(auto point : vecEachClusterPoints.at(i)){
                for(int j=i+1;j<vecEachClusterPoints.size();++j) {
                    if(vecEachClusterPoints.at(j).find(point) != vecEachClusterPoints.at(j).end()){
                        clusterDeleteFlag = true;
                    }
                    if(clusterDeleteFlag) {
                        for(auto pointIndex : vecEachClusterPoints.at(j)){
                            vecEachClusterPoints.at(i).insert(pointIndex);
                        }
                        deleteElement[j] = true;
                        flag = true;
                    }
                    clusterDeleteFlag = false;
                }
            }
        }

        std::cout<<std::endl;
        for(int i =0; i<4;++i){
            std::cout<<deleteElement[i]<<std::endl;
        }

        for(int i =0; i<vecEachClusterPoints.size();++i){
            int tmp = 0;
            if(deleteElement[i]){
                vecEachClusterPoints.erase(vecEachClusterPoints.begin() + i + tmp);
                tmp--;
            }
        }
        delete[] deleteElement;
    }while (flag);

}

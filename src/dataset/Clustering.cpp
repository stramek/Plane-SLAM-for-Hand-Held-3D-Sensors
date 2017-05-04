//
// Created by mordimer on 11.03.17.
//

#include "include/dataset/Clustering.h"

const float Clustering::MAX_ANGLE_THRESHOLD = 5.0;


float Clustering::getDistanceBetweenTwoPlanes(const Plane &firstPlane, const Plane &secondPlane){
    float distance;
    Eigen::Vector3f pointOnFirstPlane = firstPlane.computePointOnPlaneFromTwoCoordinate();

/*    distance = abs(secondPlane.getA()*pointOnFirstPlane(0) + secondPlane.getB()*pointOnFirstPlane(1) +
                   secondPlane.getC()*pointOnFirstPlane(2) - secondPlane.getD())
               / sqrtf(powf(secondPlane.getA(), 2) + sqrtf(powf(secondPlane.getB(), 2)) + sqrtf(powf(secondPlane.getC(), 2)));*/
    distance = abs(firstPlane.getD() - secondPlane.getD());

    return distance;
}

float Clustering::getAngleBetweenTwoPlanes(const Plane &firstPlane, const Plane &secondPlane){
    Eigen::Vector3f firstPlaneNormalVec = firstPlane.getPlaneNormalVec();
    Eigen::Vector3f secondPlaneNormalVec = secondPlane.getPlaneNormalVec();

    float angleCos = firstPlaneNormalVec.dot(secondPlaneNormalVec) / firstPlaneNormalVec.norm() / secondPlaneNormalVec.norm();
    if(angleCos < -1) angleCos = -1.0f;
    if(angleCos > 1) angleCos = 1.0f;
    float angle = acosf(angleCos)*180.0f/(float)M_PI;
    return angle;
}

float Clustering::getSimilarityOfTwoPlanes(const Plane &firstPlane, const Plane &secondPlane){
    float angleBetweenTwoPlanes = getAngleBetweenTwoPlanes(firstPlane, secondPlane);
/*    if(abs(angleBetweenTwoPlanes) < MAX_ANGLE_THRESHOLD){
        return getDistanceBetweenTwoPlanes(firstPlane, secondPlane);
    }
    return std::numeric_limits<float>::max();*/
    return abs(angleBetweenTwoPlanes);
}

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
                                          unsigned int *&I, const std::vector<Plane> &planesVec){
    for (unsigned int i = 0; i < planesVec.size(); ++i) {
        unsigned int indexMaxRow = 0;
        float maxSimilarityRow = std::numeric_limits<float>::max();
        for (unsigned int j = 0; j < planesVec.size(); ++j) {
            similarityMatrix[i][j].similarity = getSimilarityOfTwoPlanes(planesVec.at(i), planesVec.at(j));
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

void Clustering::computeIndexOfTwoPlanesToMerge(SimilarityItem *&nextBestMerge, unsigned int *&I,
                                                unsigned int &firstPlaneIndexToMerge,
                                                unsigned int &secondPlaneIndexToMerge, unsigned long size){
    float maxNBM_Sim = std::numeric_limits<float>::max();

    for (unsigned int j = 0; j < size; ++j) {
        if (nextBestMerge[j].similarity < maxNBM_Sim && I[j] == j) {
            maxNBM_Sim = nextBestMerge[j].similarity;
            firstPlaneIndexToMerge = j;
        }
    }

    secondPlaneIndexToMerge = I[nextBestMerge[firstPlaneIndexToMerge].index];
}

void Clustering::addNewClusterToVec(std::vector<Cluster> &clustersVec, SimilarityItem **&similarityMatrix,
                                    const unsigned int &firstPlaneIndexToMerge,
                                    const unsigned int &secondPlaneIndexToMerge){
    Cluster cluster;
    cluster.setFirstLinkIndex(firstPlaneIndexToMerge);
    cluster.setSecondLinkIndex(secondPlaneIndexToMerge);
    cluster.setDistanceBetweenLinks(similarityMatrix[firstPlaneIndexToMerge][secondPlaneIndexToMerge].similarity);
    clustersVec.push_back(cluster);
}

void Clustering::updateSimilarityMatrixState(SimilarityItem **&similarityMatrix, unsigned int *&I,
                                             const unsigned int &firstPlaneIndexToMerge,
                                             const unsigned int &secondPlaneIndexToMerge, unsigned long size){
    for (int j = 0; j < size; ++j) {
        if (I[j] == j && j != firstPlaneIndexToMerge && j != secondPlaneIndexToMerge) {
            if (similarityMatrix[firstPlaneIndexToMerge][j].similarity < similarityMatrix[secondPlaneIndexToMerge][j].similarity) {
                similarityMatrix[firstPlaneIndexToMerge][j].similarity = similarityMatrix[firstPlaneIndexToMerge][j].similarity;
                similarityMatrix[j][firstPlaneIndexToMerge].similarity = similarityMatrix[firstPlaneIndexToMerge][j].similarity;
            } else {
                similarityMatrix[firstPlaneIndexToMerge][j].similarity = similarityMatrix[secondPlaneIndexToMerge][j].similarity;
                similarityMatrix[j][firstPlaneIndexToMerge].similarity = similarityMatrix[secondPlaneIndexToMerge][j].similarity;
            }
        }
        if (I[j] == secondPlaneIndexToMerge) {
            I[j] = firstPlaneIndexToMerge;
        }
    }

}

void Clustering::updateNextBestMerge(SimilarityItem **&similarityMatrix ,SimilarityItem *&nextBestMerge, unsigned int *&I,
                                     unsigned int firstPlaneIndexToMerge,unsigned long size){
    SimilarityItem similarityMatrixItem_MaxSim;
    similarityMatrixItem_MaxSim.similarity = std::numeric_limits<float>::max();;
    for (int j = 0; j < size; ++j) {
        if (similarityMatrix[firstPlaneIndexToMerge][j].similarity < similarityMatrixItem_MaxSim.similarity && I[j] == j && j != firstPlaneIndexToMerge) {
            similarityMatrixItem_MaxSim = similarityMatrix[firstPlaneIndexToMerge][j];
        }
    }
    nextBestMerge[firstPlaneIndexToMerge] = similarityMatrixItem_MaxSim;
}

void Clustering::computeClusters(std::vector<Plane> planesVec, std::vector<Cluster> &clustersVec) {

    unsigned const long  similarityMatSize = planesVec.size();
    SimilarityItem **SimilarityMatrix;

    createSimilarityMatrix(SimilarityMatrix, similarityMatSize);

    unsigned int *I = new unsigned int[planesVec.size()];
    SimilarityItem *nextBestMerge;
    createNextBestMergeMatrix(nextBestMerge, planesVec.size());

    clusteringInitializeStep(SimilarityMatrix, nextBestMerge, I, planesVec);

    for (int i = 0; i < planesVec.size() - 1; ++i) {

        unsigned int firstPointToMergeIndex, secondPointToMergeIndex;
        computeIndexOfTwoPlanesToMerge(nextBestMerge, I, firstPointToMergeIndex, secondPointToMergeIndex,
                                       planesVec.size());
        addNewClusterToVec(clustersVec, SimilarityMatrix, firstPointToMergeIndex, secondPointToMergeIndex);

        updateSimilarityMatrixState(SimilarityMatrix, I, firstPointToMergeIndex, secondPointToMergeIndex,
                                    planesVec.size());


        updateNextBestMerge(SimilarityMatrix, nextBestMerge, I, firstPointToMergeIndex, planesVec.size());
    }
    delete[] I;
    deleteSimilarityMatrix(SimilarityMatrix, similarityMatSize);
    deleteNextBestMergeMatrix(nextBestMerge);
}

float Clustering::getDistanceBetweenTwoPoints(cv::Point_<float> point1, cv::Point_<float> point2) {
    return sqrtf(powf(point1.x - point2.x, 2) + powf(point1.y - point2.y, 2));
}

void Clustering::getClustersAfterThreshold(float cutThreshold, std::vector<Plane> pointsVec,
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

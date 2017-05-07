//
// Created by mordimer on 11.03.17.
//

#include "include/algorithms/Clustering.h"

const float Clustering::MAX_ANGLE_THRESHOLD = 5.0;

float Clustering::getDistanceBetweenPointAndPlane(Plane plane, Vector3f point) {
    float distance = abs(plane.getA() * point(0) + plane.getB() * point(1) + plane.getC() * point(2) + plane.getD()) /
                     sqrtf(powf(plane.getA(), 2) + powf(plane.getB(), 2) + powf(plane.getC(), 2));
    return distance;
}

float Clustering::getDistanceBetweenTwoPlanes(const Plane &firstPlane, const Plane &secondPlane) {
    float distance = 0;
    vector<Vector3f> pointsVec = secondPlane.getPoints();

    for (int i = 0; i < 50; ++i) {
        random_device rd;
        mt19937 rng(rd());
        uniform_int_distribution<unsigned int> pointIndex(0, secondPlane.getNumberOfPoints() - 1);
        Vector3f randomPointOnSecondPlane = pointsVec[pointIndex(rng)];
        distance += getDistanceBetweenPointAndPlane(firstPlane, randomPointOnSecondPlane);
    }

    return distance / 50;
}

float Clustering::getAngleBetweenTwoPlanes(const Plane &firstPlane, const Plane &secondPlane) {
    Eigen::Vector3f firstPlaneNormalVec = firstPlane.getPlaneNormalVec();
    Eigen::Vector3f secondPlaneNormalVec = secondPlane.getPlaneNormalVec();

    float angleCos =
            firstPlaneNormalVec.dot(secondPlaneNormalVec) / firstPlaneNormalVec.norm() / secondPlaneNormalVec.norm();
    if (angleCos < -1) angleCos = -1.0f;
    if (angleCos > 1) angleCos = 1.0f;
    float angle = acosf(angleCos) * 180.0f / (float) M_PI;
    if (angle > 90.0f) {
        angle = 180.0f - angle;
    }
    return angle;
}

float Clustering::getSimilarityOfTwoPlanes(const Plane &firstPlane, const Plane &secondPlane) {
    float angleBetweenTwoPlanes = firstPlane.getAngleBetweenTwoPlanes(secondPlane);
/*    if(abs(angleBetweenTwoPlanes) < MAX_ANGLE_THRESHOLD){
        //return getDistanceBetweenTwoPlanes(firstPlane, secondPlane);
        return abs(firstPlane.getD() - secondPlane.getD());
    }
    return std::numeric_limits<float>::max();*/
    return abs(angleBetweenTwoPlanes);
}

void Clustering::createSimilarityMatrix(SimilarityItem **&similarityMatrix, unsigned long size) {
    similarityMatrix = new SimilarityItem *[size];
    for (int i = 0; i < size; ++i) {
        similarityMatrix[i] = new SimilarityItem[size];
    }
}

void Clustering::deleteSimilarityMatrix(SimilarityItem **&similarityMatrix, unsigned long size) {
    for (int i = 0; i < size; ++i) {
        delete[] similarityMatrix[i];
    }
    delete[] similarityMatrix;
}

void Clustering::createNextBestMergeMatrix(SimilarityItem *&nextBestMerge, unsigned long size) {
    nextBestMerge = new SimilarityItem[size];
}

void Clustering::deleteNextBestMergeMatrix(SimilarityItem *&nextBestMerge) {
    delete[] nextBestMerge;
}

void Clustering::clusteringInitializeStep(SimilarityItem **&similarityMatrix, SimilarityItem *&nextBestMerge,
                                          unsigned int *&I, const std::vector<Plane> &planesVec) {
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
                                                unsigned int &secondPlaneIndexToMerge, unsigned long size) {
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
                                    const unsigned int &secondPlaneIndexToMerge) {
    Cluster cluster;
    cluster.setFirstLinkIndex(firstPlaneIndexToMerge);
    cluster.setSecondLinkIndex(secondPlaneIndexToMerge);
    cluster.setDistanceBetweenLinks(similarityMatrix[firstPlaneIndexToMerge][secondPlaneIndexToMerge].similarity);
    clustersVec.push_back(cluster);
}

void Clustering::updateSimilarityMatrixState(SimilarityItem **&similarityMatrix, unsigned int *&I,
                                             const unsigned int &firstPlaneIndexToMerge,
                                             const unsigned int &secondPlaneIndexToMerge, unsigned long size) {
    for (int j = 0; j < size; ++j) {
        if (I[j] == j && j != firstPlaneIndexToMerge && j != secondPlaneIndexToMerge) {
            if (similarityMatrix[firstPlaneIndexToMerge][j].similarity <
                similarityMatrix[secondPlaneIndexToMerge][j].similarity) {
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

void
Clustering::updateNextBestMerge(SimilarityItem **&similarityMatrix, SimilarityItem *&nextBestMerge, unsigned int *&I,
                                unsigned int firstPlaneIndexToMerge, unsigned long size) {
    SimilarityItem similarityMatrixItem_MaxSim;
    similarityMatrixItem_MaxSim.similarity = std::numeric_limits<float>::max();;
    for (int j = 0; j < size; ++j) {
        if (similarityMatrix[firstPlaneIndexToMerge][j].similarity < similarityMatrixItem_MaxSim.similarity &&
            I[j] == j && j != firstPlaneIndexToMerge) {
            similarityMatrixItem_MaxSim = similarityMatrix[firstPlaneIndexToMerge][j];
        }
    }
    nextBestMerge[firstPlaneIndexToMerge] = similarityMatrixItem_MaxSim;
}

void Clustering::computeClusters(std::vector<Plane> planesVec, std::vector<Cluster> &clustersVec) {
    std::vector<std::vector<float>> similarityMatrix;
    std::vector<bool> isClustered;
    for (unsigned long rowNumber = 0; rowNumber < planesVec.size(); ++rowNumber) {
        std::vector<float> similarityVector;
        for (unsigned long columnNumber = 0; columnNumber < planesVec.size(); ++columnNumber) {
            float planeSimilarity = getSimilarityOfTwoPlanes(planesVec.at(rowNumber), planesVec.at(columnNumber));
            similarityVector.push_back(planeSimilarity);
        }
        similarityMatrix.push_back(similarityVector);
        isClustered.push_back(false);
    }

    clustersVec.clear();

    for (unsigned int clusteringStepNum = 0; clusteringStepNum < planesVec.size() - 1; ++clusteringStepNum) {
        float maxSimilarity = std::numeric_limits<float>::max(); // most similar if value is smaller
        unsigned int firstPlaneToClusterIndex = 0;
        unsigned int secondPlaneToClusterIndex = 0;
        for (unsigned int rowNumber = 0; rowNumber < planesVec.size(); ++rowNumber) {
            for (unsigned int columnNumber = 0; columnNumber < planesVec.size(); ++columnNumber) {
                if (rowNumber != columnNumber && !isClustered.at(rowNumber) && !isClustered.at(columnNumber)) {
                    if (maxSimilarity >= similarityMatrix.at(rowNumber).at(columnNumber)) {
                        maxSimilarity = similarityMatrix.at(rowNumber).at(columnNumber);
                        firstPlaneToClusterIndex = rowNumber;
                        secondPlaneToClusterIndex = columnNumber;
                    }
                }
            }
        }
        Cluster cluster;
        cluster.setFirstLinkIndex(firstPlaneToClusterIndex);
        cluster.setSecondLinkIndex(secondPlaneToClusterIndex);
        cluster.setDistanceBetweenLinks(maxSimilarity);
        clustersVec.push_back(cluster);

        for (unsigned int columnNumber = 0; columnNumber < planesVec.size(); ++columnNumber) {
            if (similarityMatrix.at(firstPlaneToClusterIndex).at(columnNumber) <
                similarityMatrix.at(secondPlaneToClusterIndex).at(columnNumber)) {
                similarityMatrix.at(firstPlaneToClusterIndex).at(columnNumber) = similarityMatrix.at(
                        firstPlaneToClusterIndex).at(columnNumber);
                similarityMatrix.at(columnNumber).at(firstPlaneToClusterIndex) = similarityMatrix.at(
                        firstPlaneToClusterIndex).at(columnNumber);
            } else {
                similarityMatrix.at(secondPlaneToClusterIndex).at(columnNumber) = similarityMatrix.at(
                        secondPlaneToClusterIndex).at(columnNumber);
                similarityMatrix.at(columnNumber).at(secondPlaneToClusterIndex) = similarityMatrix.at(
                        secondPlaneToClusterIndex).at(columnNumber);
            }
        }
        isClustered.at(secondPlaneToClusterIndex) = true;
    }
}

float Clustering::getDistanceBetweenTwoPoints(cv::Point_<float> point1, cv::Point_<float> point2) {
    return sqrtf(powf(point1.x - point2.x, 2) + powf(point1.y - point2.y, 2));
}

void eraseFromMapByClusterIndexes(unordered_map<int, Cluster> &clusters, Cluster &cluster) {
    for (auto &index : cluster.getIndexList()) {
        clusters.erase(index);
    }
}

void insertToMapByClusterIndexes(unordered_map<int, Cluster> &clusters, Cluster &cluster) {
    for (auto &index : cluster.getIndexList()) {
        clusters.insert(pair<int, Cluster>(index, cluster));
    }
}

void Clustering::getClustersAfterThreshold(float cutThreshold, vector<Plane> planesVec,
                                           set<Cluster> &output) {
    output.clear();
    vector<Cluster> clustersVec;
    computeClusters(planesVec, clustersVec);
    sort(clustersVec.begin(), clustersVec.end());
    unordered_map<int, Cluster> clusters;

    set<int> testNumbers;
    for (Cluster cluster1 : clustersVec) {
        testNumbers.insert(cluster1.getFirstLinkIndex());
        testNumbers.insert(cluster1.getSecondLinkIndex());
    }
    cout<<"Got "<<testNumbers.size()<<" vectors inside clusters..."<<endl;

    for (Cluster &cluster : clustersVec) {
        if (cluster.getDistanceBetweenLinks() <= cutThreshold) {
            if (clusters.count(cluster.getFirstLinkIndex()) && clusters.count(cluster.getSecondLinkIndex())) {
                Cluster child1 = clusters.at(cluster.getFirstLinkIndex());
                Cluster child2 = clusters.at(cluster.getSecondLinkIndex());
                eraseFromMapByClusterIndexes(clusters, child1);
                eraseFromMapByClusterIndexes(clusters, child2);
                cluster.mergeClildrenIndexes(child1, child2);
                insertToMapByClusterIndexes(clusters, cluster);
            } else if (clusters.count(cluster.getFirstLinkIndex())) {
                Cluster child = clusters.at(cluster.getFirstLinkIndex());
                eraseFromMapByClusterIndexes(clusters, child);
                eraseFromMapByClusterIndexes(clusters, cluster);
                cluster.mergeClildrenIndexes(cluster, child);
                insertToMapByClusterIndexes(clusters, cluster);
            } else if (clusters.count(cluster.getSecondLinkIndex())) {
                Cluster child = clusters.at(cluster.getSecondLinkIndex());
                eraseFromMapByClusterIndexes(clusters, child);
                eraseFromMapByClusterIndexes(clusters, cluster);
                cluster.mergeClildrenIndexes(cluster, child);
                insertToMapByClusterIndexes(clusters, cluster);
            } else {
                clusters.insert(pair<int, Cluster>(cluster.getFirstLinkIndex(), cluster));
                clusters.insert(pair<int, Cluster>(cluster.getSecondLinkIndex(), cluster));
            }
        } else if (clusters.count(cluster.getFirstLinkIndex())) {
            cluster.getIndexList().erase(cluster.getFirstLinkIndex());
            clusters.insert(pair<int, Cluster>(cluster.getSecondLinkIndex(), cluster));
        } else if (clusters.count(cluster.getSecondLinkIndex())) {
            cluster.getIndexList().erase(cluster.getSecondLinkIndex());
            clusters.insert(pair<int, Cluster>(cluster.getFirstLinkIndex(), cluster));
        } else {
            clusters.insert(pair<int, Cluster>(cluster.getFirstLinkIndex(), cluster));
            clusters.insert(pair<int, Cluster>(cluster.getSecondLinkIndex(), cluster));
        }

    }



    set<Cluster> test;
    for (auto pair : clusters) {
        output.insert(pair.second);
        test.insert(pair.second);
    }
    cout<<"asd";
}

void Clustering::getClusteredPlaneGroup(std::vector<Plane> planesVec, vector<vector<Plane>> &clusteredPlanes) {
    set<Cluster> vecEachClusterPlanes;
    getClustersAfterThreshold(15, planesVec, vecEachClusterPlanes);
    for (auto planesIndexesInOneCluster : vecEachClusterPlanes) {
        vector<Plane> singleCluster;
        for (auto &index : planesIndexesInOneCluster.getIndexList()) {
            Plane singleClusterPlane = planesVec.at(index);
            singleCluster.push_back(singleClusterPlane);
        }
        clusteredPlanes.push_back(singleCluster);
    }
}

vector<Plane> Clustering::getAveragedPlanes(vector<vector<Plane>> &clusteredPlanes) {
    vector<Plane> averagedPlanesVec;
    for (auto planesFromCluster : clusteredPlanes) {
        float averagedD = 0;
        int averagedHue = 0;
        int averagedSaturation = 0;
        int averagedValue = 0;
        vector<Vector3f> mergedPlanePoints;
        vector<ImageCoords> mergedPlaneImageCoordsVec;
        Vector3f averagedNormalVec = Vector3f::Zero();
        for (auto plane : planesFromCluster) {
            averagedNormalVec += plane.getPlaneNormalVec();
            averagedD += plane.getD();
            averagedHue += plane.getColor().getHue();
            averagedSaturation += plane.getColor().getSaturation();
            averagedValue += plane.getColor().getValue();

            vector<Vector3f> points = plane.getPoints();
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

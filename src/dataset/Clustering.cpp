//
// Created by mordimer on 11.03.17.
//

#include "include/dataset/Clustering.h"


void Clustering::computeClasters(std::vector<cv::Point_<float>> pointsVec, std::vector<Cluster> &clastersVec) {
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
            if (maxSimilarityRow > SimilarityMatrix[i][j].similarity && i!=j) {
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
        Cluster claster;
        claster.setFirstLinkIndex(i1);
        claster.setSecondLinkIndex(i2);
        claster.setDistanceBetweenLinks(SimilarityMatrix[i1][i2].similarity);
        clastersVec.push_back(claster);

        for (int j = 0; j < pointsVec.size(); ++j) {
            if (I[j] == j && j != i1 && j != i2){
                if(SimilarityMatrix[i1][j].similarity < SimilarityMatrix[i2][j].similarity){
                    SimilarityMatrix[i1][j].similarity = SimilarityMatrix[i1][j].similarity;
                    SimilarityMatrix[j][i1].similarity = SimilarityMatrix[i1][j].similarity;
                } else{
                    SimilarityMatrix[i1][j].similarity = SimilarityMatrix[i2][j].similarity;
                    SimilarityMatrix[j][i1].similarity = SimilarityMatrix[i2][j].similarity;
                }
            }
            if(I[j] == i2){
                I[j] = i1;
            }
        }

        SimilarityMatrixItem similarityMatrixItem_MaxSim;
        similarityMatrixItem_MaxSim.similarity = std::numeric_limits<float>::max();;
        for (int j = 0; j < pointsVec.size(); ++j) {
            if (SimilarityMatrix[i1][j].similarity < similarityMatrixItem_MaxSim.similarity && I[j]==j && j!=i1) {
                similarityMatrixItem_MaxSim= SimilarityMatrix[i1][j];
            }
        }
        NBM[i1] = similarityMatrixItem_MaxSim;
    }
}

float Clustering::getDistanceBetweenTwoPoints(cv::Point_<float> point1, cv::Point_<float> point2) {
    return sqrtf(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

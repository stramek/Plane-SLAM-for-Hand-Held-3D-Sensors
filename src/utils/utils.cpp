#include "../../include/utils/utils.h"

namespace utils {

    void paintPixel(Mat &rgb, const Vector3f &vector, Vec3b color) {
        for (int i = 0 ; i < 3; ++i) {
            rgb.at<Vec3b>((int)vector[0], (int)vector[1])[i] = (uchar) color[i];
        }
    }

    pair<int, int> getRandomPosition(const Mat &mat, int areaSize) {
        random_device rd;
        mt19937 rng(rd());
        uniform_int_distribution<int> colUni((areaSize - 1) / 2, mat.cols - ((areaSize - 1) / 2) - 1);
        uniform_int_distribution<int> rowUni((areaSize - 1) / 2, mat.rows - ((areaSize - 1) / 2) - 1);
        return pair<int, int>(rowUni(rng), colUni(rng));
    }

    vector<pair<Plane, Plane>> getSimilarPlanes(const vector<Plane> &previousFrame, const vector<Plane> &currentFrame) {

        vector<pair<Plane, Plane>> toReturn;

        vector<PlaneSimilarity> planeSimilarityVec;
        //planeSimilarityVec.resize(previousFrame.size() * currentFrame.size());

        bool* previousFrameTab = new bool[previousFrame.size()];
        bool* currentFrameTab = new bool[currentFrame.size()];

        for (unsigned int i = 0; i < previousFrame.size(); ++i) {
            previousFrameTab[i] = true;
        }

        for (unsigned int i = 0; i < currentFrame.size(); ++i) {
            currentFrameTab[i] = true;
        }


        for (unsigned int i = 0; i < previousFrame.size(); ++i) {
            for (unsigned int j = 0; j < currentFrame.size(); ++j) {
                planeSimilarityVec.push_back(PlaneSimilarity(previousFrame.at(i), currentFrame.at(j)));
            }
        }

        sort(planeSimilarityVec.begin(), planeSimilarityVec.end());



        delete[] previousFrameTab;
        delete[] currentFrameTab;
        return toReturn;
    };

}
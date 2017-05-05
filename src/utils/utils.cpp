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

        for (unsigned int i = 0; i < previousFrame.size(); ++i) {
            for (unsigned int j = 0; j < currentFrame.size(); ++j) {
                planeSimilarityVec.push_back(PlaneSimilarity(previousFrame.at(i), currentFrame.at(j), i, j));
            }
        }

        sort(planeSimilarityVec.begin(), planeSimilarityVec.end());

        for (PlaneSimilarity &outerPlaneSimilarity : planeSimilarityVec) {
            if (!outerPlaneSimilarity.isAnyOfFramesTaken()) {
                if (outerPlaneSimilarity.isSimilarityValid()) {
                    toReturn.push_back(pair<Plane, Plane>(outerPlaneSimilarity.getLastFrame(), outerPlaneSimilarity.getCurrentFrame()));

                    for (PlaneSimilarity &innerPlaneSimilarity : planeSimilarityVec) {
                        if (innerPlaneSimilarity.isOneOfIndexesEqual(outerPlaneSimilarity)) {
                            innerPlaneSimilarity.setFramesAsTaken();
                        }
                    }
                } else {
                    break;
                }
            }
        }

        return toReturn;
    };

    void fillPlaneVector(int numberOfPoints, int areaSize, ImagePair &imagePair, vector<Plane> &planeVector) {
        planeVector.clear();
        for (int iteration = 0; iteration < numberOfPoints; ++iteration) {
            pair<int, int> position = utils::getRandomPosition(imagePair.getDepth(), areaSize);
            ImageCoords imageCoords = ImageCoords(position, areaSize);
            Mat rgb = imagePair.getRgb();
            Mat croppedImage = rgb(Rect(imageCoords.getUpLeftX(),
                                        imageCoords.getUpLeftY(),
                                        imageCoords.getAreaSize(),
                                        imageCoords.getAreaSize()));

            vector<Vector3f> pointsVector;
            for (int i = imageCoords.getUpLeftY(); i <= imageCoords.getDownRightY(); ++i) {
                for (int j = imageCoords.getUpLeftX(); j <= imageCoords.getDownRightX(); ++j) {
                    pointsVector.push_back(Vector3f(i, j, imagePair.getDepthAt(i, j)));
                }
            }

            Plane plane = PlanePca::getPlane(pointsVector, croppedImage, imageCoords);
            Vec3b color = plane.isValid() ? Vec3b(0, 255, 0) : Vec3b(0, 0, 255);
            for (Vector3f vector : pointsVector) {
                utils::paintPixel((Mat &) imagePair.getRgb(), vector, color);
            }
            if (plane.isValid()) {
                planeVector.push_back(plane);
            }
        }
    }

    void mergePlanes(vector<Plane> &planeVector){
        vector<vector<Plane>> clusteredPLanes;
        Clustering::getClusteredPlaneGroup(planeVector, clusteredPLanes);
        planeVector = Clustering::getAveragedPlanes(clusteredPLanes);

    }

    void displayClusteredPlanes(ImagePair &imagePair, vector<Plane> plane){
        vector<vector<Plane>> clusteredPLanes;
        Clustering::getClusteredPlaneGroup(plane, clusteredPLanes);
        int i = 0;
        for (auto singleCluster : clusteredPLanes){
            ++i;
            for(auto planeInCluster : singleCluster){
                putText(imagePair.getRgb(), to_string(i), Point(planeInCluster.getImageCoords().getCenterX(), planeInCluster.getImageCoords().getCenterY()),FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            }
        }
        imshow("Clustered planes", imagePair.getRgb());
    }

}
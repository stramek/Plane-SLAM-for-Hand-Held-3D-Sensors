#include <include/models/PointCloud.h>
#include "include/utils/planeUtils.h"

namespace planeUtils {
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
                    toReturn.push_back(pair<Plane, Plane>(outerPlaneSimilarity.getLastFrame(),
                                                          outerPlaneSimilarity.getCurrentFrame()));

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

    void fillPlaneVector(int numberOfPoints, int areaSize, ImagePair &imagePair, vector<Plane> *planeVector,
                         vector<Plane> *previousPlaneVector, float previousPlanePercent, bool colorPlanes) {
        planeVector->clear();

        if (previousPlaneVector != nullptr) {
            auto engine = default_random_engine{};
            shuffle(previousPlaneVector->begin(), previousPlaneVector->end(), engine);
        }

        for (int iteration = 0; iteration < numberOfPoints; ++iteration) {

            ImageCoords imageCoords;

            pair<int, int> position;
            if (previousPlaneVector != nullptr && iteration < numberOfPoints * previousPlanePercent
                && iteration < previousPlaneVector->size()) {
                imageCoords = previousPlaneVector->at(iteration).getImageCoords();
            } else {
                position = utils::getRandomPosition(imagePair.getDepth(), areaSize);
                imageCoords = ImageCoords(position, areaSize);
            }

            Mat rgb = imagePair.getRgb();
            Mat depth = imagePair.getDepth();
            Mat croppedRgbImage = rgb(Rect(imageCoords.getUpLeftX(),
                                        imageCoords.getUpLeftY(),
                                        imageCoords.getAreaSize(),
                                        imageCoords.getAreaSize()));

            Mat croppedDepthImage = depth(Rect(imageCoords.getUpLeftX(),
                                           imageCoords.getUpLeftY(),
                                           imageCoords.getAreaSize(),
                                           imageCoords.getAreaSize()));

            PointCloud pointCloud;

            pointCloud.depth2cloud(croppedDepthImage, croppedRgbImage, imageCoords.getUpLeftX(), imageCoords.getUpLeftY());

            vector<Vector3f> pointsVector = pointCloud.getPoints();

            std::cout << "fillPlaneVector1" << std::endl;
            Plane plane = PlanePca::getPlane(pointsVector, croppedRgbImage, imageCoords);
            std::cout << "fillPlaneVector2" << std::endl;
            if (colorPlanes) {
                Vec3b color = plane.isValid() ? Vec3b(0, 255, 0) : Vec3b(0, 0, 255);
                croppedRgbImage.setTo(color);
            }
            if (plane.isValid()) {
                planeVector->push_back(plane);
            }
        }
    }

    void visualizeSimilarPlanes(vector<pair<Plane, Plane>> &similarPlanes, const Mat &previousImage,
                                const Mat &currentImage, int limitPoints) {
        Size previousImageSize = previousImage.size();
        Size currentImageSize = currentImage.size();
        Mat merged(previousImageSize.height, previousImageSize.width + currentImageSize.width, CV_8UC3);
        Mat left(merged, Rect(0, 0, previousImageSize.width, previousImageSize.height));
        previousImage.copyTo(left);
        Mat right(merged, Rect(previousImageSize.width, 0, currentImageSize.width, currentImageSize.height));
        currentImage.copyTo(right);

        RNG rng(12345);
        int pointNumber = 0;
        for (pair<Plane, Plane> pair : similarPlanes) {

            ImageCoords previousImageCoords = pair.first.getImageCoords();
            ImageCoords currentImageCoords = pair.second.getImageCoords();

            Point previousPlanePoint = Point(previousImageCoords.getCenterX(), previousImageCoords.getCenterY());
            Point currentPlanePoint = Point(previousImageSize.width + currentImageCoords.getCenterX(),
                                            currentImageCoords.getCenterY());

            int size = previousImageCoords.getAreaSize() / 2;
            Scalar color = Scalar(rng.uniform(100, 255), rng.uniform(100, 255), rng.uniform(100, 255));

            circle(merged, previousPlanePoint, size, color, 1);
            circle(merged, currentPlanePoint, size, color, 1);
            line(merged, previousPlanePoint, currentPlanePoint, color, 1);

            pointNumber++;
            if (pointNumber >= limitPoints) break;
        }

        imshow("Merged", merged);

        imwrite( "../images/similar.png", merged );

        waitKey();
    }

    void filterPairsByAngle(vector<pair<Plane, Plane>> &pairs) {
        cout<<endl<<endl<<endl;
        for (auto it = pairs.begin(); it != pairs.end();) {
            cout<<"Angle is: "<<it->first.getAngleBetweenTwoPlanes(it->second)<<"... ";
            if (it->first.getAngleBetweenTwoPlanes(it->second) > MAX_ANGLE_BETWEEN_PLANES) {
                cout<<"deleteing."<<endl;
                it = pairs.erase(it++);
            } else {
                cout<<"it's fine."<<endl;
                ++it;
            }
        }
        // TODO: WTF
    }

    void mergePlanes(vector<Plane> &planeVector) {
        if (planeVector.size() == 0) return;
        vector<vector<Plane>> clusteredPLanes;
        //Clustering::getClusteredPlaneGroup(planeVector, clusteredPLanes);
        planeVector = Clustering::getAveragedPlanes(clusteredPLanes);
    }

    void displayClusteredPlanes(ImagePair &imagePair, vector<Plane> planes) {
        if (planes.size() == 0) return;
        vector<vector<Plane>> clusteredPLanes;
        Clustering clustering;
        clustering.setCutSimilarity(5.0);
        clustering.selectParts(planes, clusteredPLanes);
        //Clustering::getClusteredPlaneGroup(plane, clusteredPLanes);
        int i = 0;
        for (auto singleCluster : clusteredPLanes) {
            ++i;
            for (auto planeInCluster : singleCluster) {
                putText(imagePair.getRgb(), to_string(i), Point(planeInCluster.getImageCoords().getCenterX(),
                                                                planeInCluster.getImageCoords().getCenterY()),
                        FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            }
        }
        imshow("Clustered planes", imagePair.getRgb());
        //waitKey();
    }
}
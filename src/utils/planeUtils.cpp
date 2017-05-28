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
                    if (outerPlaneSimilarity.isAngleBetweenPlanedValid()) {
                        toReturn.push_back(pair<Plane, Plane>(outerPlaneSimilarity.getLastFrame(),
                                                              outerPlaneSimilarity.getCurrentFrame()));

                        for (PlaneSimilarity &innerPlaneSimilarity : planeSimilarityVec) {
                            if (innerPlaneSimilarity.isOneOfIndexesEqual(outerPlaneSimilarity)) {
                                innerPlaneSimilarity.setFramesAsTaken();
                            }
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
                         vector<Plane> *previousPlaneVector, double previousPlanePercent, bool colorPlanes) {
        if (previousPlanePercent > 1.0 || previousPlanePercent < 0.0)
            throw runtime_error("previousPlanePercent value must be between <0.0 ; 1.0>");
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

            pointCloud.depth2cloud(croppedDepthImage, croppedRgbImage, imageCoords.getUpLeftX(),
                                   imageCoords.getUpLeftY());

            vector<Point3D> pointsVector = pointCloud.getPoints3D();

            //cout<<"Getting plane: "<<iteration + 1<<" of "<<numberOfPoints<<endl;
            Plane plane = PlanePca::getPlane(pointsVector, croppedRgbImage, imageCoords);

            if (colorPlanes) {
                Vec3b color = plane.isValid() ? Vec3b(0, 255, 0) : Vec3b(0, 0, 255);
                croppedRgbImage.setTo(color);
            }
            if (plane.isValid()) {
                planeVector->push_back(plane);
            }
        }
    }

    void
    fillPlaneVector(int numberOfPoints, int areaSize, vector<Plane> *planeVector, vector<Plane> *previousPlaneVector,
                    double previousPlanePercent,
                    libfreenect2::Registration *registration, libfreenect2::Frame *undistorted,
                    libfreenect2::Frame *registered) {

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
                position = utils::getRandomPosition((int) undistorted->width, (int) undistorted->height, areaSize);
                imageCoords = ImageCoords(position, areaSize);
            }

            bool shouldBreak = false;
            long nanPixelsCount = 0;
            PointCloud pointCloud;
            for (int row = imageCoords.getUpLeftY(); row < imageCoords.getDownRightY() && !shouldBreak; ++row) {
                for (int col = imageCoords.getUpLeftX(); col < imageCoords.getDownRightX() && !shouldBreak; ++col) {
                    float x, y, z, color;
                    registration->getPointXYZRGB(undistorted, registered, row, col, x, y, z, color);
                    const uint8_t *p = reinterpret_cast<uint8_t *>(&color);

                    if (!isnanf(z) && color != 0) {
                        Point3D point3D(-x, -y, -z, p[2], p[1], p[0]);
                        pointCloud.push_back(point3D);
                    } else if (imageCoords.hasTooMuchNanPixels(++nanPixelsCount)) {
                        pointCloud.clear();
                        shouldBreak = true;
                    }
                }
            }

            //cout<< shouldBreak << " points: "<<pointCloud.getPoints3D().size()<<endl;

            vector<Point3D> pointsVector = pointCloud.getPoints3D();
            vector<Point3D> points = pointCloud.getPoints3D();
            Plane plane = PlanePca::getPlane(pointsVector, points, imageCoords);
            if (plane.isValid()) {
                planeVector->push_back(plane);
            }
        }
    }

    Mat getRGBFrameMat(libfreenect2::Registration *registration, libfreenect2::Frame *undistorted,
                       libfreenect2::Frame *registered) {
        Mat toReturn(Size(registered->width, registered->height), CV_8UC3);
        toReturn = 0;
        for (int row = 0; row < registered->height; ++row) {
            for (int col = 0; col < registered->width; ++col) {
                float x, y, z, color;
                registration->getPointXYZRGB(undistorted, registered, row, col, x, y, z, color);
                const uint8_t *p = reinterpret_cast<uint8_t *>(&color);
                if (!isnanf(z)) {
                    Vec3b color1(p[0], p[1], p[2]);
                    toReturn.at<Vec3b>(Point(col, row)) = color1;
                }
            }
        }
        return toReturn;
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
            Scalar color = Scalar(rng.uniform(180, 255), rng.uniform(180, 255), rng.uniform(180, 255));

            circle(merged, previousPlanePoint, size, color, 2);
            circle(merged, currentPlanePoint, size, color, 2);
            line(merged, previousPlanePoint, currentPlanePoint, color, 2);

            double angle = pair.first.getAngleBetweenTwoPlanes(pair.second);
            int colorDiff = abs(pair.first.getColor().getHue() - pair.second.getColor().getHue());
            stringstream stream;
            stream << fixed << setprecision(1) << angle;
            Point centerPoint = ((previousPlanePoint + currentPlanePoint) / 2);
            centerPoint.x = centerPoint.x - 50;
            putText(merged, "Angle: " + stream.str() + " Color diff: " + to_string(colorDiff), centerPoint, FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

            pointNumber++;
            if (pointNumber >= limitPoints) break;
        }

        imshow("Merged", merged);

        //imwrite("../images/similar" + to_string(SCREENSHOT_HELPER++) +".png", merged);

        waitKey(1);
    }

    void mergePlanes(vector<Plane> &planeVector) {
        if (planeVector.size() == 0) return;
        vector<vector<Plane>> clusteredPLanes;
        Clustering clustering;
        clustering.setCutSimilarity(CLUSTERING_MAX_DISTANCE_THRESHOLD);
        clustering.selectParts(planeVector, clusteredPLanes);
        planeVector = Clustering::getAveragedPlanes(clusteredPLanes);
    }

    void displayClusteredPlanes(ImagePair &imagePair, vector<Plane> planes) {
        if (planes.size() == 0) return;
        vector<vector<Plane>> clusteredPLanes;
        Clustering clustering;
        clustering.setCutSimilarity(CLUSTERING_MAX_DISTANCE_THRESHOLD);
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
        //imshow("Clustered planes", imagePair.getRgb());
        //waitKey();
    }
}
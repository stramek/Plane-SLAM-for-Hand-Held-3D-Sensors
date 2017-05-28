//
// Created by stramek on 28.05.17.
//

#include <include/models/PlaneFiller.h>

void PlaneFiller::setNumberOfPoints(int numberOfPoints) {
    PlaneFiller::numberOfPoints = numberOfPoints;
}

void PlaneFiller::setAreaSize(int areaSize) {
    PlaneFiller::areaSize = areaSize;
}

void PlaneFiller::setImagePair(ImagePair *imagePair) {
    PlaneFiller::imagePair = imagePair;
}

void PlaneFiller::setPreviousPlanePercent(double previousPlanePercent) {
    PlaneFiller::previousPlanePercent = previousPlanePercent;
}

void PlaneFiller::setColorPlanes(bool colorPlanes) {
    PlaneFiller::colorPlanes = colorPlanes;
}

void PlaneFiller::setVectorToFill(vector<Plane> *vectorToFill) {
    PlaneFiller::vectorToFill = vectorToFill;
}

void PlaneFiller::setPreviousVector(vector<Plane> *previousVector) {
    PlaneFiller::previousVector = previousVector;
}

bool PlaneFiller::isPrevousVectorPassed() {
    return previousVector != nullptr;
}

const ImageCoords PlaneFiller::getNextCoords(int iteration) {
    pair<int, int> position;
    if (previousVector != nullptr && iteration < numberOfPoints * previousPlanePercent
        && iteration < previousVector->size()) {
        return previousVector->at((unsigned int) iteration).getImageCoords();
    } else {
        if (getFillerMode() == KINECT) {
            position = utils::getRandomPosition((int) undistorted->width, (int) undistorted->height, areaSize);
        } else if (getFillerMode() == DATASET) {
            position = utils::getRandomPosition(imagePair->getDepth(), areaSize);
        }
        return ImageCoords(position, areaSize);
    }
};

const Mat PlaneFiller::getCroppedMat(Mat &image, ImageCoords &imageCoords) {
    return image(Rect(imageCoords.getUpLeftX(),
                      imageCoords.getUpLeftY(),
                      imageCoords.getAreaSize(),
                      imageCoords.getAreaSize()));
}

void PlaneFiller::fillVector(vector<Plane> *vectorToFill) {
    vectorToFill->clear();

    if (isPrevousVectorPassed()) {
        shuffle(previousVector->begin(), previousVector->end(), default_random_engine{});
    }

    if (getFillerMode() == PlaneFiller::DATASET) {
        fillVectorFromDataset(vectorToFill);
    } else if (getFillerMode() == PlaneFiller::KINECT) {
        fillVectorFromKinect(vectorToFill);
    }
}

void PlaneFiller::fillVectorFromDataset(vector<Plane> *vectorToFill) {
    Mat rgb = imagePair->getRgb();
    Mat depth = imagePair->getDepth();

    for (int iteration = 0; iteration < numberOfPoints; ++iteration) {

        ImageCoords imageCoords = getNextCoords(iteration);
        Mat croppedRgbImage = getCroppedMat(rgb, imageCoords);
        Mat croppedDepthImage = getCroppedMat(depth, imageCoords);

        PointCloud pointCloud;
        pointCloud.depth2cloud(croppedDepthImage, croppedRgbImage, imageCoords.getUpLeftX(), imageCoords.getUpLeftY());
        vector<Point3D> pointsVector = pointCloud.getPoints3D();

        Plane plane = PlanePca::getPlane(pointsVector, croppedRgbImage, imageCoords);

        if (colorPlanes) {
            Vec3b color = plane.isValid() ? Vec3b(0, 255, 0) : Vec3b(0, 0, 255);
            croppedRgbImage.setTo(color);
        }
        if (plane.isValid()) {
            vectorToFill->push_back(plane);
        }
    }
}

void PlaneFiller::fillVectorFromKinect(vector<Plane> *vectorToFill) {
    for (int iteration = 0; iteration < numberOfPoints; ++iteration) {
        ImageCoords imageCoords = getNextCoords(iteration);
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
        vector<Point3D> pointsVector = pointCloud.getPoints3D();
        Plane plane = PlanePca::getPlane(pointsVector, imageCoords);
        if (plane.isValid()) {
            vectorToFill->push_back(plane);
        }
    }
}

int PlaneFiller::getNumberOfPoints() const {
    return numberOfPoints;
}

int PlaneFiller::getAreaSize() const {
    return areaSize;
}

const ImagePair *PlaneFiller::getImagePair() const {
    return imagePair;
}

double PlaneFiller::getPreviousPlanePercent() const {
    return previousPlanePercent;
}

bool PlaneFiller::isColorPlanes() const {
    return colorPlanes;
}

vector<Plane> *PlaneFiller::getVectorToFill() const {
    return vectorToFill;
}

vector<Plane> *PlaneFiller::getPreviousVector() const {
    return previousVector;
}

libfreenect2::Registration *PlaneFiller::getRegistration() const {
    return registration;
}

void PlaneFiller::setRegistration(libfreenect2::Registration *registration) {
    PlaneFiller::registration = registration;
}

libfreenect2::Frame *PlaneFiller::getUndistorted() const {
    return undistorted;
}

void PlaneFiller::setUndistorted(libfreenect2::Frame *undistorted) {
    PlaneFiller::undistorted = undistorted;
}

libfreenect2::Frame *PlaneFiller::getRegistered() const {
    return registered;
}

void PlaneFiller::setRegistered(libfreenect2::Frame *registered) {
    PlaneFiller::registered = registered;
}

void PlaneFillerBuilder::validateData() {
    if (planeFiller.get()->getFillerMode() == PlaneFiller::ERROR) {
        throw runtime_error("You need to call withKinect() or withDataset() method before build()");
    }
    if (planeFiller.get()->getPreviousPlanePercent() > 1.0 || planeFiller.get()->getPreviousPlanePercent() < 0.0) {
        throw runtime_error("previousPlanePercent value must be between <0.0 ; 1.0>");
    }
    if (planeFiller.get()->getAreaSize() % 2 == 0) {
        throw runtime_error("Passed areaSize need to be odd number!");
    }
}
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

void PlaneFiller::fillVector(vector<Plane> *vectorToFill) {
    vectorToFill->clear();

    if (previousVector != nullptr) {
        auto engine = default_random_engine{};
        shuffle(previousVector->begin(), previousVector->end(), engine);
    }

    for (int iteration = 0; iteration < numberOfPoints; ++iteration) {

        ImageCoords imageCoords;

        pair<int, int> position;
        if (previousVector != nullptr && iteration < numberOfPoints * previousPlanePercent
            && iteration < previousVector->size()) {
            imageCoords = previousVector->at(iteration).getImageCoords();
        } else {
            position = utils::getRandomPosition(imagePair->getDepth(), areaSize);
            imageCoords = ImageCoords(position, areaSize);
        }

        Mat rgb = imagePair->getRgb();
        Mat depth = imagePair->getDepth();
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

        vector<Vector3d> pointsVector = pointCloud.getPoints();

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

void PlaneFillerBuilder::validateData() {
    if (planeFiller.get()->getImagePair() == nullptr) {
        throw runtime_error("Passed imagePair is empty!");
    }
    if (planeFiller.get()->getPreviousPlanePercent() > 1.0 || planeFiller.get()->getPreviousPlanePercent() < 0.0) {
        throw runtime_error("previousPlanePercent value must be between <0.0 ; 1.0>");
    }
}

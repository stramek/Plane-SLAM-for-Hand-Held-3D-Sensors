//
// Created by stramek on 28.05.17.
//

#ifndef PROJEKTMAGISTERSKI_PLANEFILLER_H
#define PROJEKTMAGISTERSKI_PLANEFILLER_H

#include "ImagePair.h"
#include "Plane.h"
#include <string>
#include <iostream>
#include <memory>
#include <include/algorithms/PlaneRansac.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <include/models/PointCloud.h>
#include "include/utils/utils.h"
#include "include/models/PlaneDetector.h"

using namespace std;

class PlaneFiller {
public:

    void setNumberOfPoints(int numberOfPoints);

    void setAreaSize(int areaSize);

    void setImagePair(ImagePair *imagePair);

    void fillVector(vector<Plane> *vectorToFill);

    void setVectorToFill(vector<Plane> *vectorToFill);

    void setPreviousVector(vector<Plane> *previousVector);

    void setPreviousPlanePercent(double previousPlanePercent);

    void setColorPlanes(bool colorPlanes);

    int getNumberOfPoints() const;

    int getAreaSize() const;

    const ImagePair *getImagePair() const;

    double getPreviousPlanePercent() const;

    bool isColorPlanes() const;

    vector<Plane> *getVectorToFill() const;

    vector<Plane> *getPreviousVector() const;

    libfreenect2::Registration *getRegistration() const;

    void setRegistration(libfreenect2::Registration *registration);

    libfreenect2::Frame *getUndistorted() const;

    void setUndistorted(libfreenect2::Frame *undistorted);

    libfreenect2::Frame *getRegistered() const;

    void setRegistered(libfreenect2::Frame *registered);

    void setPlaneDetector(PlaneDetector *planeDetector);

    PlaneDetector *getPlaneDetector() const;

    virtual ~PlaneFiller();

    enum FillerMode {
        DATASET, KINECT, ERROR
    };

    bool isPrevousVectorPassed();

    FillerMode getFillerMode() {
        if (registration != nullptr && imagePair != nullptr) return FillerMode::ERROR;
        if (registration != nullptr) return FillerMode::KINECT;
        else if (imagePair != nullptr) return FillerMode::DATASET;
        return FillerMode::ERROR;
    }

private:

    int numberOfPoints = 100;
    int areaSize = 15;
    ImagePair *imagePair = nullptr;
    double previousPlanePercent = 0.0;
    bool colorPlanes = false;

    vector<Plane> *vectorToFill;
    vector<Plane> *previousVector = nullptr;
    libfreenect2::Registration *registration = nullptr;
    libfreenect2::Frame *undistorted = nullptr;
    libfreenect2::Frame *registered = nullptr;

    PlaneDetector *planeDetector = nullptr;

    const ImageCoords getNextCoords(int iteration);

    const Mat getCroppedMat(Mat &image, ImageCoords &imageCoords);

    void fillVectorFromDataset(vector<Plane> *vectorToFill);

    void fillVectorFromKinect(vector<Plane> *vectorToFill);
};

class PlaneFillerBuilder {
public:

    PlaneFillerBuilder() {
        planeFiller = make_unique<PlaneFiller>();
    }

    virtual ~PlaneFillerBuilder() {};

    PlaneFiller *build() {
        validateData();
        return planeFiller.release();
    }

    PlaneFillerBuilder *withPlaneDetector(PlaneDetector *planeDetector) {
        planeFiller->setPlaneDetector(planeDetector);
        return this;
    }

    PlaneFillerBuilder *withNumberOfPoints(int numberOfPoints) {
        planeFiller->setNumberOfPoints(numberOfPoints);
        return this;
    }

    PlaneFillerBuilder *withAreaSize(int areaSize) {
        planeFiller->setAreaSize(areaSize);
        return this;
    }

    PlaneFillerBuilder *withDataset(ImagePair *imagePair) {
        planeFiller->setImagePair(imagePair);
        return this;
    }

    PlaneFillerBuilder *withColorPlanes(bool colorPlanes) {
        planeFiller->setColorPlanes(true);
        return this;
    }

    PlaneFillerBuilder *withKinect(libfreenect2::Registration *registration, libfreenect2::Frame *undistoreted,
                                   libfreenect2::Frame *registered) {
        planeFiller->setRegistration(registration);
        planeFiller->setUndistorted(undistoreted);
        planeFiller->setRegistered(registered);
        return this;
    }

    PlaneFillerBuilder *withPreviousPlanePercent(vector<Plane> *previousVector, double previousPlanePercent) {
        planeFiller->setPreviousVector(previousVector);
        planeFiller->setPreviousPlanePercent(previousPlanePercent);
        return this;
    }

protected:
    unique_ptr<PlaneFiller> planeFiller;

private:
    void validateData();
};

#endif //PROJEKTMAGISTERSKI_PLANEFILLER_H

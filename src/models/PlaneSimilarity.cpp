//
//  PlaneSimilarity.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 02.05.2017.
//
//

#include "include/models/PlaneSimilarity.h"

PlaneSimilarity::PlaneSimilarity(const Plane &lastFrame, const Plane &currentFrame, unsigned int lastFrameIndex,
                                 unsigned int currentFrameIndex) : lastFrame(lastFrame), currentFrame(currentFrame),
                                                                   lastFrameIndex(lastFrameIndex),
                                                                   currentFrameIndex(currentFrameIndex) {
    calculateSimilarity();
}

void PlaneSimilarity::calculateSimilarity() {
    similarity = abs(currentFrame.getColor().getHue() - lastFrame.getColor().getHue());
}

int PlaneSimilarity::getSimilarity() const {
    return similarity;
}

unsigned int PlaneSimilarity::getLastFrameIndex() const {
    return lastFrameIndex;
}

unsigned int PlaneSimilarity::getCurrentFrameIndex() const {
    return currentFrameIndex;
}

Plane &PlaneSimilarity::getLastFrame() {
    return lastFrame;
}

Plane &PlaneSimilarity::getCurrentFrame() {
    return currentFrame;
}

bool PlaneSimilarity::isAnyOfFramesTaken() const {
    return lastFrameTaken || currentFrameTaken;
}

void PlaneSimilarity::setFramesAsTaken() {
    PlaneSimilarity::currentFrameTaken = true;
    PlaneSimilarity::lastFrameTaken = true;
}

bool PlaneSimilarity::isOneOfIndexesEqual(PlaneSimilarity &planeSimilarity) const {
    return lastFrameIndex == planeSimilarity.getLastFrameIndex() ||
           currentFrameIndex == planeSimilarity.getCurrentFrameIndex();
}

bool PlaneSimilarity::isSimilarityValid() const {
    return similarity <= MAX_SIMILARITY_VALUE;
}

bool PlaneSimilarity::isAngleBetweenPlanedValid() {
    return getLastFrame().getAngleBetweenTwoPlanes(getCurrentFrame()) < MAX_ANGLE_BETWEEN_PLANES;
}

bool PlaneSimilarity::isLastFrameTaken() const {
    return lastFrameTaken;
}

bool PlaneSimilarity::isCurrentFrameTaken() const {
    return currentFrameTaken;
}

PlaneSimilarity::PlaneSimilarity() {}

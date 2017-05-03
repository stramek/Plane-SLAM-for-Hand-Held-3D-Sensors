//
//  PlaneSimilarity.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 02.05.2017.
//
//

#include "include/utils/PlaneSimilarity.h"

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

const Plane &PlaneSimilarity::getLastFrame() const {
    return lastFrame;
}

const Plane &PlaneSimilarity::getCurrentFrame() const {
    return currentFrame;
}

bool PlaneSimilarity::isAnyOfFramesTaken() const {
    return lastFrameTaken || currentFrameTaken;
}

void PlaneSimilarity::setFramesAsTaken() {
    PlaneSimilarity::currentFrameTaken = true;
    PlaneSimilarity::lastFrameTaken = true;
}

bool PlaneSimilarity::isOneOfIndexesEqual(PlaneSimilarity planeSimilarity) const {
    return lastFrameIndex == planeSimilarity.getLastFrameIndex() ||
           currentFrameIndex == planeSimilarity.getCurrentFrameIndex();
}

bool PlaneSimilarity::isSimilarityValid() const {
    return similarity <= MAX_SIMILARITY_VALUE;
}

//
//  PlaneSimilarity.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 02.05.2017.
//
//

#include "../../include/utils/PlaneSimilarity.h"

PlaneSimilarity::PlaneSimilarity(const Plane &lastFrame, const Plane &currentFrame) : lastFrame(lastFrame),
                                                                                      currentFrame(currentFrame) {
    calculateSimilarity();
}

void PlaneSimilarity::calculateSimilarity() {
    similarity = abs(currentFrame.getColor().getHue() - lastFrame.getColor().getHue());
}

int PlaneSimilarity::getSimilarity() const {
    return similarity;
}

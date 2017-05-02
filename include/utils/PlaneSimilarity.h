//
//  PlaneSimilarity.hpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 02.05.2017.
//
//

#ifndef PlaneSimilarity_h
#define PlaneSimilarity_h
#ifdef __GNUC__
// Avoid tons of warnings with root code
#pragma GCC system_header
#endif

#include "include/models/Plane.h"
#include "include/utils/constants.h"
#include <math.h>
using namespace std;

class PlaneSimilarity {
public:
    PlaneSimilarity(const Plane &lastFrame, const Plane &currentFrame, unsigned int lastFrameIndex,
                    unsigned int currentFrameIndex);

    int getSimilarity() const;
    const Plane &getLastFrame() const;
    const Plane &getCurrentFrame() const;
    unsigned int getLastFrameIndex() const;
    unsigned int getCurrentFrameIndex() const;
    bool isAnyOfFramesTaken() const;
    void setFramesAsTaken();
    bool isOneOfIndexesEqual(PlaneSimilarity planeSimilarity) const;
    bool isSimilarityValid() const;

    bool operator < (const PlaneSimilarity& plane) const {
        return (similarity < plane.similarity);
    }
private:
    Plane lastFrame, currentFrame;
    int similarity = numeric_limits<int>::max();
    unsigned int lastFrameIndex, currentFrameIndex;
    bool lastFrameTaken = false;
    bool currentFrameTaken = false;
    void calculateSimilarity();
};

#endif /* PlaneSimilarity_h */

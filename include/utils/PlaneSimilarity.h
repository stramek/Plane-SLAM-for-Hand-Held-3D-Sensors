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
#include <math.h>

class PlaneSimilarity {
public:
    PlaneSimilarity(const Plane &lastFrame, const Plane &currentFrame);
    int getSimilarity() const;

    bool operator < (const PlaneSimilarity& plane) const {
        return (similarity < plane.similarity);
    }
private:
    Plane lastFrame, currentFrame;
    int similarity = -1;
    void calculateSimilarity();
};

#endif /* PlaneSimilarity_h */

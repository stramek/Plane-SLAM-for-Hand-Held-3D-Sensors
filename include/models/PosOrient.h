//
// Created by stramek on 15.07.17.
//

#ifndef PROJEKTMAGISTERSKI_POSORIENT_H
#define PROJEKTMAGISTERSKI_POSORIENT_H

#include <Eigen/Dense>
#include <iostream>
#include "math.h"

using namespace Eigen;
using namespace std;

class PosOrient {
public:
    PosOrient(const Vector3d &position, const Vector4d &orientation);
    void setPosOrient(const Vector7d &posOrient);
    void print();
    void printDiff(const PosOrient &posOrient);
private:
    Vector3d position;
    Vector4d orientation;
};

#endif //PROJEKTMAGISTERSKI_POSORIENT_H

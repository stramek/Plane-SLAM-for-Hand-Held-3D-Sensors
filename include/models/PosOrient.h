//
// Created by stramek on 15.07.17.
//

#ifndef PROJEKTMAGISTERSKI_POSORIENT_H
#define PROJEKTMAGISTERSKI_POSORIENT_H

#include <Eigen/Dense>
#include <iostream>
#include "math.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/vertex_se3_quat.h"
#include "g2o/types/slam3d/vertex_plane_quat.h"
#include "g2o/types/slam3d/edge_se3_plane.h"

using namespace Eigen;
using namespace std;

class PosOrient {
public:
    PosOrient();
    PosOrient(const Vector3d &position, const Vector4d &orientation);
    void setPosOrient(const g2o::Vector7d &posOrient);
    const void print() const;
    void printDiff(const PosOrient &posOrient);
    Quaterniond getQuaternion() const;
    Vector3d getPosition() const;
    pair<Vector3d, Vector3d> minus(PosOrient &posOrient);
private:
    Vector3d position;
    Vector4d orientation;

    Vector3d fromRotationMat(const Matrix<double, 3, 3>& pose) const;
};

#endif //PROJEKTMAGISTERSKI_POSORIENT_H
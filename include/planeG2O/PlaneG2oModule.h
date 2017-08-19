//
// Created by stramek on 04.06.17.
//

#ifndef PROJEKTMAGISTERSKI_PLANEG2OMODULE_H
#define PROJEKTMAGISTERSKI_PLANEG2OMODULE_H

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
//#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "include/models/Plane.h"
#include "include/models/PosOrient.h"

using namespace g2o;
using namespace std;

class PlaneG2oModule {
public:
    PosOrient ComputeCameraPos(vector<pair<Plane, Plane>> &matchedPlanes);
    Eigen::Quaterniond normAndDToQuat(double d, Eigen::Vector3d norm);

private:

    const int PLANES_INDEXES_SHIFT = 100000;
};

#endif //PROJEKTMAGISTERSKI_PLANEG2OMODULE_H

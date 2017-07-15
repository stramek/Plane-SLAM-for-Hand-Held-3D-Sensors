//
// Created by stramek on 04.06.17.
//

#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/slam3d/vertex_se3_quat.h>
#include <Eigen/Geometry>
#include <g2o/types/slam3d/vertex_plane_quat.h>
#include "include/planeG2O/PlaneG2oModule.h"

using namespace Eigen;

Eigen::Quaterniond normAndDToQuat(double d, Eigen::Vector3d norm){
    Eigen::Quaterniond res;
    norm.normalize();
    res.x() = norm[0];
    res.y() = norm[1];
    res.z() = norm[2];
    res.w() = -d;

    g2o::VertexPlaneQuat::normalizeAndUnify(res);
    return res;
}

PlaneG2oModule::PlaneG2oModule() {
    g2o::BlockSolverX::LinearSolverType* linearSolverMin = new g2o::LinearSolverPCG<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX* solverMin = new g2o::BlockSolverX(linearSolverMin);
    g2o::OptimizationAlgorithmLevenberg* algorithmMin = new g2o::OptimizationAlgorithmLevenberg(solverMin);
    optimizerMin.setAlgorithm(algorithmMin);

    //set init camera pose
    g2o::VertexSE3Quat* curV = new g2o::VertexSE3Quat();
    Vector3d trans(0.0, 0.0, 0.0);
    Quaterniond q;
    q.setIdentity();
    g2o::SE3Quat poseSE3Quat(q, trans);
    curV->setEstimate(poseSE3Quat);
    curV->setId(0);
    curV->setFixed(true);

    optimizerMin.addVertex(curV);
}

PlaneG2oModule &PlaneG2oModule::getInstance() {
    static PlaneG2oModule instance;
    return instance;
}

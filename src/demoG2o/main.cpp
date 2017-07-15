#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>
#include <stdint.h>

#include <unordered_set>
#include <include/demoG2o/main.h>

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/vertex_se3_quat.h"
#include "g2o/types/slam3d/vertex_plane_quat.h"
#include "g2o/types/slam3d/edge_se3_plane.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#include "g2o/types/slam3d/isometry3d_gradients.h"
#include "g2o/types/slam3d/dquat2mat.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"

using namespace Eigen;
using namespace std;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d)
G2O_USE_TYPE_GROUP(slam3d)

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

int main(int argc, const char *argv[]) {

    g2o::SparseOptimizer optimizerMin;
    {
        g2o::BlockSolverX::LinearSolverType* linearSolverMin = new g2o::LinearSolverPCG<g2o::BlockSolverX::PoseMatrixType>();
        g2o::BlockSolverX* solverMin = new g2o::BlockSolverX(linearSolverMin);
        g2o::OptimizationAlgorithmLevenberg* algorithmMin = new g2o::OptimizationAlgorithmLevenberg(solverMin);

        optimizerMin.setAlgorithm(algorithmMin);
    }
    // set camera pose
    g2o::VertexSE3Quat* curV = new g2o::VertexSE3Quat();

    Vector3d trans(0.0, 0.0, 0.0);
    Quaterniond q;
    q.setIdentity();
    g2o::SE3Quat poseSE3Quat(q, trans);
    curV->setEstimate(poseSE3Quat);
    curV->setId(0);
    curV->setFixed(true);

    optimizerMin.addVertex(curV);

    // set camera pose in second frame
    g2o::VertexSE3Quat* curV1 = new g2o::VertexSE3Quat();

    curV1->setEstimate(poseSE3Quat);
    curV1->setId(1);

    optimizerMin.addVertex(curV1);


    // set plane pose

    std::vector<std::vector<Plane>> frameVector;
    std::vector<Plane> planeFrame1 = {Plane(-5, Vector3d(1, 0, 0)), Plane(-2, Vector3d(0, 1, 0)),
                                        Plane(10, Vector3d(0, 0, 1))};
    std::vector<Plane> planeFrame2 = {Plane(-10, Vector3d(1, 0, 0)), Plane(-2, Vector3d(0, 1, 0)),
                                      Plane(10, Vector3d(0, 0, 1))};
    frameVector.push_back(planeFrame1);
    frameVector.push_back(planeFrame2);


    for(int i=0; i<planeFrame1.size(); ++i){
        g2o::VertexPlaneQuat* curV2 = new g2o::VertexPlaneQuat();

        curV2->setEstimate(normAndDToQuat(planeFrame1.at(i).getD(), planeFrame1.at(i).getPlaneNormalVec()));
        curV2->setId( 2  + i);
//				if(pl == 0 || pl == 1 || pl == 2){
//					curV->setFixed(true);
//				}
//				curV->setMarginalized(true);
        optimizerMin.addVertex(curV2);
    }

    for(int j=0; j<2;++j){
        for(int i=0; i<planeFrame2.size(); ++i){
            g2o::EdgeSE3Plane* curEdge = new g2o::EdgeSE3Plane();
            curEdge->setVertex(0, optimizerMin.vertex(j));
            curEdge->setVertex(1, optimizerMin.vertex(planeFrame1.size() + i));

            if (j == 0) {
                curEdge->setMeasurement(normAndDToQuat(planeFrame1.at(i).getD(), planeFrame1.at(i).getPlaneNormalVec()));
            } else {
                curEdge->setMeasurement(normAndDToQuat(planeFrame2.at(i).getD(), planeFrame2.at(i).getPlaneNormalVec()));
            }

            curEdge->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

            optimizerMin.addEdge(curEdge);
        }
    }

    optimizerMin.save("before.g2o");
    cout << "optimizerMin.vertices().size() = " << optimizerMin.vertices().size() << endl;
    cout << "optimizerMin.edges.size() = " << optimizerMin.edges().size() << endl;
    optimizerMin.initializeOptimization();
    cout << "optimization initialized" << endl;
    optimizerMin.setVerbose(true);
    optimizerMin.optimize(10);
    cout << "optimized" << endl;
    optimizerMin.save("after.g2o");

    return 1;
}

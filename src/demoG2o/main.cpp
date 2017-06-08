#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>
#include <stdint.h>

#include <unordered_set>
#include <include/demoG2o/main.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
//#include "g2o/math_groups/se3quat.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"



#include "g2o/types/slam3d/vertex_plane_quat.h"
#include "g2o/types/slam3d/edge_se3_plane.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"

using namespace Eigen;
using namespace std;

int main(int argc, const char *argv[]) {

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *blockSolver = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);

    optimizer.setVerbose(true);
    optimizer.setAlgorithm(optimizationAlgorithm);


    g2o::Factory* factory = g2o::Factory::instance();


    g2o::ParameterSE3Offset *cameraOffset = new g2o::ParameterSE3Offset;
    cameraOffset->setId(0);
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;  R  << 1,  0,  0,  0,  1,  0,  0, 0,  1;
    cameraPose= R; cameraPose.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
    cameraOffset->setOffset(cameraPose);
    optimizer.addParameter(cameraOffset);

    int vertex_id = 0;
    for (size_t i = 0; i < 15; ++i) {
        Vector3d trans(i * 0.04 - 1., 0, 0);

        Eigen::Quaterniond q;
        q.setIdentity();
        g2o::SE3Quat pose(q, trans);
        g2o::VertexSE3Expmap *v_se3 = new g2o::VertexSE3Expmap();
        v_se3->setId(vertex_id);
        if (i < 2) {
            v_se3->setFixed(true);
        }
        v_se3->setEstimate(pose);
        optimizer.addVertex(v_se3);
        vertex_id++;
        std::cout<< vertex_id << std::endl;
    }

    optimizer.save("test.g2o");
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);

    optimizer.optimize(10);
    optimizer.save("after.g2o");
    return 1;
}

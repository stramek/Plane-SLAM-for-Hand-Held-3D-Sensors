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
    //if (DENSE) {
    linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolverX::PoseMatrixType>();
    /*} else {
        linearSolver
                = new g2o::LinearSolverCholmod<g2o
        ::BlockSolver_6_3::PoseMatrixType>();
    }*/


    g2o::BlockSolverX *solver_ptr
            = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    g2o::Factory* factory = g2o::Factory::instance();

    //double focal_length = 1000.;
    g2o::ParameterSE3Offset parameterSE3Offset;
    Eigen::Isometry3d cameraPose(Eigen::Isometry3d::Identity());
/*    Eigen::Matrix3d R; R << 1, 0, 0,
                            0, 1, 0,
                            0, 0, 1;*/
    //cameraPose = R;
    //cameraPose.translation() = cam;
    parameterSE3Offset.setOffset(cameraPose);
    //Vector2d principal_point(320., 240.);

    //vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > true_poses;
    //g2o::CameraParameters * cam_params = new g2o::CameraParameters (focal_length, principal_point, 0.);
    //cam_params->setId(0);

    if (!optimizer.addParameter(&parameterSE3Offset)) {
        assert(false);
    }

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
}

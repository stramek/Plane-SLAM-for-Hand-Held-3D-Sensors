#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>
#include <stdint.h>

#include <unordered_set>
#include <include/demoG2o/main.h>

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include "g2o/types/slam3d/parameter_se3_offset.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"

#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"



//#include "g2o/types/slam3d/vertex_plane_quat.h"
//#include "g2o/types/slam3d/edge_se3_plane.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"


using namespace Eigen;
using namespace std;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d)
G2O_USE_TYPE_GROUP(slam3d)

int main(int argc, const char *argv[]) {

    g2o::BlockSolverX::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX* blockSolver;
    blockSolver = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);

    g2o::SparseOptimizer optimizer;

    optimizer.setVerbose(true);
    optimizer.setAlgorithm(optimizationAlgorithm);

    g2o::Factory* factory;
    factory = g2o::Factory::instance();

    g2o:: ParameterSE3Offset* cameraOffset;
    cameraOffset = new g2o::ParameterSE3Offset;
    cameraOffset->setId(0);
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;  R  << 1,  0,  0,  0,  1,  0,  0,  0, 1;
    //Quaternion q(-0.5, 0.5, -0.5, 0.5);
    cameraPose= R; cameraPose.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
    cameraOffset->setOffset(cameraPose);
    optimizer.addParameter(cameraOffset);

    int vertex_id = 0;
    std::cout<<"test\n";
    for (size_t i = 0; i < 3; ++i) {
        Vector3d trans(i * 0.04 - 1., 0, 0);
        Eigen::Quaterniond q;

        q.setIdentity();
        g2o::SE3Quat pose(q, trans);
        g2o::VertexSE3 *v_se3 = new g2o::VertexSE3();

        v_se3->setId(vertex_id);
        if (i < 1) {
            v_se3->setFixed(true);
        }
        v_se3->setEstimate(pose);
        if (!optimizer.addVertex(v_se3))
            std::cout << "Something wrong\n";
        vertex_id++;
        std::cout<< vertex_id << std::endl;
    }

    // measurement between vertexid 0 and vertexid 1
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->setId(100);
    edge->setMeasurement(Isometry3d::Identity());
    g2o::OptimizableGraph::Vertex* from = optimizer.vertex(0);
    g2o::OptimizableGraph::Vertex* to = optimizer.vertex(1);
    edge->setVertex(0, from);
    edge->setVertex(1, to);
    optimizer.addEdge(edge);

    // measurement between vertexid 1 and vertexid 2
    g2o::EdgeSE3* edge1 = new g2o::EdgeSE3();
    edge1->setId(101);
    edge1->setMeasurement(Isometry3d::Identity());
    from = optimizer.vertex(1);
    to = optimizer.vertex(2);
    edge1->setVertex(0, from);
    edge1->setVertex(1, to);
    optimizer.addEdge(edge1);

    // another measurement between vertexid 0 and vertexid 1
    g2o::EdgeSE3* edge2 = new g2o::EdgeSE3();
    edge2->setId(102);
    Isometry3d measure(Isometry3d::Identity());
    measure(0,3)=0.1;
    edge2->setMeasurement(measure);
    from = optimizer.vertex(0);
    to = optimizer.vertex(1);
    edge2->setVertex(0, from);
    edge2->setVertex(1, to);
    optimizer.addEdge(edge2);

    optimizer.save("test.g2o");
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(10);
    optimizer.save("after.g2o");
    return 1;
}

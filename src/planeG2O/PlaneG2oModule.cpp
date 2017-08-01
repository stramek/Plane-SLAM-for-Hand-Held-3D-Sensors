//
// Created by stramek on 04.06.17.
//

#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/slam3d/vertex_se3_quat.h>
#include <Eigen/Geometry>
#include <g2o/types/slam3d/vertex_plane_quat.h>
#include <g2o/types/slam3d/edge_se3_plane.h>
#include "include/planeG2O/PlaneG2oModule.h"
#include "include/models/PosOrient.h"

using namespace Eigen;

Eigen::Quaterniond PlaneG2oModule::normAndDToQuat(double d, Eigen::Vector3d norm){
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
    positionNumber = 0;

    g2o::BlockSolverX::LinearSolverType* linearSolverMin = new g2o::LinearSolverPCG<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX* solverMin = new g2o::BlockSolverX(linearSolverMin);
    g2o::OptimizationAlgorithmLevenberg* algorithmMin = new g2o::OptimizationAlgorithmLevenberg(solverMin);
    optimizerMin.setAlgorithm(algorithmMin);

    //set init camera pose
    g2o::VertexSE3Quat* curV = new g2o::VertexSE3Quat();
    Vector3d trans(0.0, 0.0, -2.5);
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

void PlaneG2oModule::ComputeCameraPos(vector<pair<Plane, Plane>> &matchedPlanes) {
    if(matchedPlanes.size() > 2){
        positionNumber++;
        g2o::VertexSE3Quat* curV = new g2o::VertexSE3Quat();
        Vector3d trans(0.0, 0.0, 0.0);
        Quaterniond q;
        q.setIdentity();
        g2o::SE3Quat poseSE3Quat(q, trans);
        curV->setEstimate(poseSE3Quat);
        curV->setId(positionNumber);
        optimizerMin.addVertex(curV);


        for(int i=0; i<matchedPlanes.size(); ++i){
            g2o::VertexPlaneQuat* curV2 = new g2o::VertexPlaneQuat();

            curV2->setEstimate(normAndDToQuat(matchedPlanes.at(i).first.getD(), matchedPlanes.at(i).first.getPlaneNormalVec()));
            curV2->setId(PLANES_INDEXES_SHIFT + i);

            optimizerMin.addVertex(curV2);
        }


        for(int j=0; j<2; ++j){
            for(int i=0; i<matchedPlanes.size(); ++i){
                std::cout<< std::endl << std::endl;
                g2o::EdgeSE3Plane* curEdge = new g2o::EdgeSE3Plane();
                curEdge->setVertex(0, optimizerMin.vertex(j));
                curEdge->setVertex(1, optimizerMin.vertex(PLANES_INDEXES_SHIFT + i));
                if(j == 0){
                    matchedPlanes.at(i).first.print();
                    matchedPlanes.at(i).second.print();
                    curEdge->setMeasurement(normAndDToQuat(matchedPlanes.at(i).first.getD(), matchedPlanes.at(i).first.getPlaneNormalVec()));
                }
                else {
                    curEdge->setMeasurement(normAndDToQuat(matchedPlanes.at(i).second.getD(), matchedPlanes.at(i).second.getPlaneNormalVec()));
                }

                curEdge->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

                optimizerMin.addEdge(curEdge);
            }
        }

        optimizerMin.save("before.g2o");
        optimizerMin.initializeOptimization();
        cout << "optimization initialized" << endl;
        optimizerMin.setVerbose(true);
        optimizerMin.optimize(50000);
        optimizerMin.save("after.g2o");
        cout << "optimization ended" << endl;


        PosOrient posOrient[2];
        for (int i = 0; i < 2; ++i) {
            g2o::VertexSE3Quat* curPoseVert = static_cast<g2o::VertexSE3Quat*>(optimizerMin.vertex(i));
            g2o::Vector7d poseVect = curPoseVert->estimate().toVector();
            posOrient[i].setPosOrient(poseVect);
            posOrient[i].print();
        }
    }
    else{
        std::cout<<"Unable to compute transformation between two frames. Number of matched planes less than 3!" << endl;
    }
}

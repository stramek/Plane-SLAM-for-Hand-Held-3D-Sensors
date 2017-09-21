//
// Created by stramek on 25.08.17.
//

#include <include/planeG2O/MatchPlanesG2o.h>
#include "include/planeG2O/GlobalG2oMap.h"

g2o::SparseOptimizer optimizerMin;

Eigen::Quaterniond GlobalG2oMap::normAndDToQuat(double d, Eigen::Vector3d norm) {
    Eigen::Quaterniond res;
    norm.normalize();
    res.x() = norm[0];
    res.y() = norm[1];
    res.z() = norm[2];
    res.w() = -d;

    g2o::VertexPlaneQuat::normalizeAndUnify(res);
    return res;
}

GlobalG2oMap::GlobalG2oMap() {}

void GlobalG2oMap::addNewFrames(vector<Plane> &planes) {
    HELPER_ITERATION++;
    if (!initialized) {
        initializeFirstFrame(planes);
        initialized = true;
    } else {
        addNextFramePlanes(planes);
    }
}

void GlobalG2oMap::initializeFirstFrame(vector<Plane> &planes) {
    loadFile();
    positionNumber = 0;

    g2o::BlockSolverX::LinearSolverType *linearSolverMin = new g2o::LinearSolverPCG<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solverMin = new g2o::BlockSolverX(linearSolverMin);
    g2o::OptimizationAlgorithmLevenberg *algorithmMin = new g2o::OptimizationAlgorithmLevenberg(solverMin);
    optimizerMin.setAlgorithm(algorithmMin);

    //set init camera pose
    g2o::VertexSE3Quat *curV = new g2o::VertexSE3Quat();
    Vector3d trans(0.0, 0.0, 0.0);
    Quaterniond q;
    q.setIdentity();
    g2o::SE3Quat poseSE3Quat(q, trans);
    curV->setEstimate(poseSE3Quat);
//    cout<<"Adding VertexSE3Quat id = " << CAMERA_POS_INDEXES_SHIFT + positionNumber <<endl;
    curV->setId(CAMERA_POS_INDEXES_SHIFT + positionNumber);
    curV->setFixed(true);

    optimizerMin.addVertex(curV);

    for (Plane &plane : planes) {

        tuple<long, bool, Plane> status = GlobalMap::getInstance().addPlaneToMap(plane, lastPosOrient, positionNumber);

        if (get<1>(status)) {
            //add planes to graph
            g2o::VertexPlaneQuat *curV2 = new g2o::VertexPlaneQuat();
            curV2->setEstimate(normAndDToQuat(plane.getD(), plane.getPlaneNormalVec()));
            cout<<"Adding VertexPlaneQuat id = " << get<0>(status) <<endl;
            curV2->setId((int) get<0>(status));
            optimizerMin.addVertex(curV2);

            //add edge to graph
            g2o::EdgeSE3Plane *curEdge = new g2o::EdgeSE3Plane();
            curEdge->setVertex(0, optimizerMin.vertex(CAMERA_POS_INDEXES_SHIFT + positionNumber));
            curEdge->setVertex(1, optimizerMin.vertex((int) get<0>(status)));

            curEdge->setMeasurement(normAndDToQuat(plane.getD(), plane.getPlaneNormalVec()));

            curEdge->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

            optimizerMin.addEdge(curEdge);
        }
    }
}

void GlobalG2oMap::addNextFramePlanes(vector<Plane> &planes) {
    positionNumber++; // increment camera position number

    //set init camera pose
    g2o::VertexSE3Quat *curV = new g2o::VertexSE3Quat();
    Vector3d trans(lastPosOrient.getPosition());
    Quaterniond res(lastPosOrient.getQuaternion());
    g2o::SE3Quat poseSE3Quat(res, trans);
    curV->setEstimate(poseSE3Quat);
    //cout<<"Adding VertexSE3Quat id = " << CAMERA_POS_INDEXES_SHIFT + positionNumber <<endl;
    curV->setId(CAMERA_POS_INDEXES_SHIFT + positionNumber);

    optimizerMin.addVertex(curV);

    matchedPlanes.clear();
    vector<Plane> unmatchedPlanes;

    if (planeUtils::arePlanesValid(planes)) {
        MatchPlanesG2o matchPlanesG2o;
        matchPlanesG2o.setLastPosOrient(lastPosOrient);
        vector<Plane> globalPlanes = GlobalMap::getInstance().getGlobalMapVector();
        matchedPlanes = matchPlanesG2o.getSimilarPlanes(globalPlanes, planes);
        unmatchedPlanes = matchPlanesG2o.getUnmatchedPlanes();
    }



    // DEBUG


   /* vector<Plane> unmatchedPlanesGlobal;
    for (Plane &plane:unmatchedPlanes) {
        unmatchedPlanesGlobal.push_back(plane.getPlaneSeenFromGlobalCamera(lastPosOrient));
    }*/


    //DEBUG

    /*if(matchedPlanes.size() < 3) {
        cout<<"*********************************************"<<endl;
        cout<<"FRAME TAKEN FROM BELTER !!!"<<endl;
        g2o::EdgeSE3 *edgeSE3 = new g2o::EdgeSE3();
        edgeSE3->setVertex(0, optimizerMin.vertex(CAMERA_POS_INDEXES_SHIFT));
        edgeSE3->setVertex(1, optimizerMin.vertex(positionNumber + CAMERA_POS_INDEXES_SHIFT));
        PosOrient currentPosOrient = pointSlamPosOrients.at(positionNumber);
        Vector3d position = currentPosOrient.getPosition();
        position[2] = position[2] - G2O_Z_OFFSET;
        g2o::SE3Quat poseSE3Quat1(currentPosOrient.getQuaternion(), position);
        edgeSE3->setMeasurement(poseSE3Quat1);
        edgeSE3->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
        optimizerMin.addEdge(edgeSE3);
        currentPosOrient.print();
        cout<<"*********************************************"<<endl;

    } else {*/
        for (auto &planePair : matchedPlanes) {
            //add edge to graph
            g2o::EdgeSE3Plane *curEdge = new g2o::EdgeSE3Plane();
            curEdge->setVertex(0, optimizerMin.vertex(CAMERA_POS_INDEXES_SHIFT + positionNumber));
            curEdge->setVertex(1, optimizerMin.vertex(planePair.first.getId()));

            curEdge->setMeasurement(normAndDToQuat(planePair.second.getD(), planePair.second.getPlaneNormalVec()));

            curEdge->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

            optimizerMin.addEdge(curEdge);
        }


        for(auto &plane : unmatchedPlanes) {
            tuple<long, bool, Plane> status = GlobalMap::getInstance().addPlaneToMap(plane, lastPosOrient, positionNumber);

            if (get<1>(status)) {
                g2o::VertexPlaneQuat *curV2 = new g2o::VertexPlaneQuat();
                curV2->setEstimate(normAndDToQuat(get<2>(status).getD(), get<2>(status).getPlaneNormalVec()));
                //cout<<"Adding VertexPlaneQuat id = " << get<0>(status) <<endl;
                curV2->setId((int) get<0>(status));
                optimizerMin.addVertex(curV2);

                //add edge to graph
                g2o::EdgeSE3Plane *curEdge = new g2o::EdgeSE3Plane();
                curEdge->setVertex(0, optimizerMin.vertex(CAMERA_POS_INDEXES_SHIFT + positionNumber));
                curEdge->setVertex(1, optimizerMin.vertex((int) get<0>(status)));

                curEdge->setMeasurement(normAndDToQuat(plane.getD(), plane.getPlaneNormalVec()));

                curEdge->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

                optimizerMin.addEdge(curEdge);
            }


        }
    //}

    optimizerMin.setVerbose(false);
    optimizerMin.initializeOptimization();
    optimizerMin.optimize(100);

    g2o::VertexSE3Quat *curPoseVert = static_cast<g2o::VertexSE3Quat *>(optimizerMin.vertex(
            CAMERA_POS_INDEXES_SHIFT + positionNumber));
    g2o::Vector7d poseVect = curPoseVert->estimate().toVector();
    lastPosOrient.setPosOrient(poseVect);

    for (int i = 0; i < GlobalMap::getInstance().getCurrentId(); ++i) {
        g2o::VertexPlaneQuat *curPlaneVert = static_cast<g2o::VertexPlaneQuat *>(optimizerMin.vertex(i));
        if (curPlaneVert == nullptr) continue;
        Eigen::Quaterniond quaternion = curPlaneVert->estimate();
        Eigen::Vector3d normVec(quaternion.x(), quaternion.y(), quaternion.z());
        double norm = normVec.norm();
        normVec.normalize();
        Plane plane(-quaternion.w() / norm, normVec);
        plane.setId(curPlaneVert->id());
        GlobalMap::getInstance().updatePlane(plane);
    }
    string s = "output" + to_string(positionNumber) + ".g2o";
    optimizerMin.save(s.c_str());

//    ImageLoader imageLoader(500);
//    imageLoader.setCurrentPhoto(positionNumber);
//    ImagePair currentFrame = imageLoader.getNextPair();
//    Mat prevMat = currentFrame.getRgb();
//    Mat curMat = imageLoader.getNextPair(120).getRgb();
//    planeUtils::visualizeSimilarPlanes(matchedPlanes, prevMat, curMat);
//    //planeUtils::visualizePlaneLocations(GlobalMap::getInstance().getGlobalMapVector(), unmatchedPlanesGlobal, currentFrame.getRgb(), currentFrame.getRgb());
//    waitKey();

    PosOrient posOrient(Vector3d(0.301, 0.0047, -0.114), Vector4d(0.027, 0.312, 0.123, 0.942));
    posOrient.print();

    vector<Plane> asd;
    for (Plane &plane : planes) {
        asd.push_back(plane.getPlaneSeenFromGlobalCamera(posOrient));
    }

    ImageLoader imageLoader(500);
    imageLoader.setCurrentPhoto(1);
    ImagePair currentFrame = imageLoader.getNextPair();
    QGLVisualizer visualizer;
    visualizer.updateCloud(currentFrame.getRgb(), currentFrame.getDepth());
    visualizer.updatePlanes(asd);
    visualizer.setWindowTitle("visualizer");
    visualizer.setPHCPModel(PHCP_MODEL);
    visualizer.show();

    cv::namedWindow("asd");
    waitKey();
}

const PosOrient &GlobalG2oMap::getLastPosOrient() const {
    return lastPosOrient;
}

void GlobalG2oMap::saveTrajectoryToFile() {
    ofstream trajectoryFile;
    trajectoryFile.open("trajectories/trajectory.txt", ios::trunc | ios::out);
    for (int i = 0; i <= positionNumber; ++i){
        g2o::VertexSE3Quat *curPoseVert = static_cast<g2o::VertexSE3Quat *>(optimizerMin.vertex(
                CAMERA_POS_INDEXES_SHIFT + i));
        g2o::Vector7d poseVect = curPoseVert->estimate().toVector();
        trajectoryFile << poseVect[0] << " " << poseVect[1] << " " << poseVect[2] << " " << poseVect[6] << " "
                       << poseVect[3] << " " << poseVect[4] << " " << poseVect[5] << "\n";
    }
    trajectoryFile.close();
}

const vector<pair<Plane, Plane>> &GlobalG2oMap::getMatchedPlanes() const {
    return matchedPlanes;
}

void GlobalG2oMap::printLastPoseOrient() {
    cout<<"Number of planes inside global map: " << GlobalMap::getInstance().getGlobalMapPlanes().size() <<endl;
    lastPosOrient.print();
}

void GlobalG2oMap::loadFile() {
    ifstream source;
    source.open("../point_features/graphKabsch_g2o_BAuncert99.g2o", ios_base::in);
    for(std::string line; std::getline(source, line); )
    {
        string type;
        std::istringstream in(line);
        in >> type;

        if(type == "VERTEX_SE3:QUAT")
        {
            int index;
            Vector7d poseVect;
            in >> index;
            for(int i=0; i<7; ++i){
                in >> poseVect[i];
            }
            PosOrient posOrient;
            posOrient.setPosOrient(poseVect);
            pointSlamPosOrients.push_back(posOrient);
        }
    }
}
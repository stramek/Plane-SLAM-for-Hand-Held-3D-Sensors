//
// Created by stramek on 25.08.17.
//

#include <include/planeG2O/MatchPlanesG2o.h>
#include "include/planeG2O/GlobalG2oMap.h"


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

GlobalG2oMap::GlobalG2oMap() {
}

void GlobalG2oMap::addNewFrames(vector<Plane> &planes) {
    if (!initialized) {
        initializeFirstFrame(planes);
        initialized = true;
    } else {
        addNextFramePlanes(planes);
        cout<<"Number of planes inside global map: "<<GlobalMap::getInstance().getGlobalMapPlanes().size()<<endl;
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
    curV->setId(positionNumber);
    curV->setFixed(true);

    optimizerMin.addVertex(curV);

    for (Plane &plane : planes) {

        tuple<long, bool, Plane> status = GlobalMap::getInstance().addPlaneToMap(plane, lastPosOrient, positionNumber);

        if (get<1>(status)) {
            //add planes to graph
            g2o::VertexPlaneQuat *curV2 = new g2o::VertexPlaneQuat();
            curV2->setEstimate(normAndDToQuat(plane.getD(), plane.getPlaneNormalVec()));
            cout<<"Adding VertexPlaneQuat id = " << get<0>(status) <<endl;
            curV2->setId((int) get<0>(status) + CAMERA_POS_INDEXES_SHIFT);
            optimizerMin.addVertex(curV2);

            //add edge to graph
            g2o::EdgeSE3Plane *curEdge = new g2o::EdgeSE3Plane();
            curEdge->setVertex(0, optimizerMin.vertex(positionNumber));
            curEdge->setVertex(1, optimizerMin.vertex((int) get<0>(status) + CAMERA_POS_INDEXES_SHIFT));

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
    //trans[2] = trans[2] + G2O_Z_OFFSET;
    Quaterniond res(lastPosOrient.getQuaternion());
    g2o::SE3Quat poseSE3Quat(res, trans);
    curV->setEstimate(poseSE3Quat);
    //cout<<"Adding VertexSE3Quat id = " << CAMERA_POS_INDEXES_SHIFT + positionNumber <<endl;
    curV->setId(positionNumber);

    optimizerMin.addVertex(curV);

    MatchPlanesG2o matchPlanesG2o;
    matchPlanesG2o.setLastPosOrient(lastPosOrient);


    vector<Plane> globalPlanes = GlobalMap::getInstance().getGlobalMapVector();
    for (Plane &plane: globalPlanes) {
        plane.print();
    }
    cout<<"----------------------------------"<<endl;
    cout<<"Global plane size: "<<globalPlanes.size()<<endl;
    cout<<"Current plane size: "<<planes.size()<<endl;
    vector<pair<Plane, Plane>> matchedPlanes = matchPlanesG2o.getSimilarPlanes(globalPlanes, planes);

    ImageLoader imageLoader(60);
    ImagePair prevPair = imageLoader.getNextPair();
    ImagePair currentFrame = imageLoader.getNextPair(50);

    planeUtils::visualizeSimilarPlanes(matchedPlanes, prevPair.getRgb(), currentFrame.getRgb());
    planeUtils::visualizePlaneLocations(globalPlanes, planes, currentFrame.getRgb(), currentFrame.getRgb());

    waitKey();


    if(matchedPlanes.size() < 3) {
        g2o::EdgeSE3 *edgeSE3 = new g2o::EdgeSE3();
        edgeSE3->setVertex(0, optimizerMin.vertex(0));
        edgeSE3->setVertex(1, optimizerMin.vertex(positionNumber));
        PosOrient currentPosOrient = pointSlamPosOrients.at(positionNumber);
        Vector3d position = currentPosOrient.getPosition();
        position[2] = position[2] - G2O_Z_OFFSET;
        g2o::SE3Quat poseSE3Quat(currentPosOrient.getQuaternion(), position);
        edgeSE3->setMeasurement(poseSE3Quat);

        edgeSE3->setInformation(Eigen::Matrix<double, 6, 6>::Identity());

        optimizerMin.addEdge(edgeSE3);

        std::ofstream out;

        // std::ios::app is the open mode "append" meaning
        // new data will be written to the end of the file.
        out.open("indexes.txt", std::ios::app);

        out << positionNumber << std::endl;

        out.close();

    } else {
        vector<Plane> unmatchedPlanes = matchPlanesG2o.getUnmatchedPlanes();
        for (Plane &plane: unmatchedPlanes) {
            plane.print();
        }
        cout<<"Unmatched planes size: "<<unmatchedPlanes.size()<<endl;


//    vector<Plane> framePlanesConv;

        //cout << "Initial lastPosOrient is: " << endl;
        //lastPosOrient.print();

        for (auto &planePair : matchedPlanes) {
            //add edge to graph
            g2o::EdgeSE3Plane *curEdge = new g2o::EdgeSE3Plane();
            curEdge->setVertex(0, optimizerMin.vertex(positionNumber));
            curEdge->setVertex(1, optimizerMin.vertex(planePair.first.getId() + CAMERA_POS_INDEXES_SHIFT));

            curEdge->setMeasurement(normAndDToQuat(planePair.second.getD(), planePair.second.getPlaneNormalVec()));

            curEdge->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

            optimizerMin.addEdge(curEdge);
        }


        for(auto &plane : unmatchedPlanes) {
            tuple<long, bool, Plane> status = GlobalMap::getInstance().addPlaneToMap(plane, lastPosOrient, positionNumber);

            g2o::VertexPlaneQuat *curV2 = new g2o::VertexPlaneQuat();
            curV2->setEstimate(normAndDToQuat(get<2>(status).getD(), get<2>(status).getPlaneNormalVec()));
            //cout<<"Adding VertexPlaneQuat id = " << get<0>(status) <<endl;
            curV2->setId((int) get<0>(status) + CAMERA_POS_INDEXES_SHIFT);
            curV2->setId((int) get<0>(status) + CAMERA_POS_INDEXES_SHIFT);
            optimizerMin.addVertex(curV2);

            //add edge to graph
            g2o::EdgeSE3Plane *curEdge = new g2o::EdgeSE3Plane();
            curEdge->setVertex(0, optimizerMin.vertex(positionNumber));
            curEdge->setVertex(1, optimizerMin.vertex((int) get<0>(status) + CAMERA_POS_INDEXES_SHIFT));

            curEdge->setMeasurement(normAndDToQuat(plane.getD(), plane.getPlaneNormalVec()));

            curEdge->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

            optimizerMin.addEdge(curEdge);
        }
    }


    optimizerMin.setVerbose(false);
    optimizerMin.initializeOptimization();
    optimizerMin.optimize(100);

    g2o::VertexSE3Quat *curPoseVert = static_cast<g2o::VertexSE3Quat *>(optimizerMin.vertex(positionNumber));
    g2o::Vector7d poseVect = curPoseVert->estimate().toVector();
    //poseVect[2] = poseVect[2] - G2O_Z_OFFSET;
    //poseVect = -poseVect;
    //poseVect[6] = -poseVect[6];
    lastPosOrient.setPosOrient(poseVect);
    cout << endl << "Setting new lastPosOrient to..." << endl;
    lastPosOrient.print();

    for (int i = 0; i < GlobalMap::getInstance().getCurrentId(); ++i) {
        g2o::VertexPlaneQuat *curPlaneVert = static_cast<g2o::VertexPlaneQuat *>(optimizerMin.vertex(i + CAMERA_POS_INDEXES_SHIFT));
        Eigen::Quaterniond quaternion = curPlaneVert->estimate();

        //quaternion.normalize();
        /*double eps = 1e-9;
        if((quaternion.w() < 0.0 ||
           (fabs(quaternion.w()) < eps && quaternion.z() < 0.0) ||
           (fabs(quaternion.w()) < eps && fabs(quaternion.z()) < eps && quaternion.y() < 0.0) ||
           (fabs(quaternion.w()) < eps && fabs(quaternion.z()) < eps && fabs(quaternion.y()) < eps && quaternion.x() < 0.0)))
        {
            quaternion.coeffs() = -quaternion.coeffs();
        }*/

        Eigen::Vector3d normVec(quaternion.x(), quaternion.y(), quaternion.z());
        double norm = normVec.norm();
        normVec.normalize();
        Plane plane(-quaternion.w() / norm, normVec);
        plane.setId(curPlaneVert->id() - CAMERA_POS_INDEXES_SHIFT);
        GlobalMap::getInstance().updatePlane(plane);
    }
    string s = "output" + to_string(positionNumber) + ".g2o";
    optimizerMin.save(s.c_str());

    QGLVisualizer visu;
    visu.setWindowTitle("Point in cluster");
    visu.visualizeVectorRot(matchedPlanes, lastPosOrient);
    visu.show();

    namedWindow("asd");
    waitKey();
}

const PosOrient &GlobalG2oMap::getLastPosOrient() const {
    return lastPosOrient;
}

void GlobalG2oMap::saveTrajectoryToFile() {
    ofstream trajectoryFile;
    trajectoryFile.open("trajectories/trajectory.txt", ios::trunc | ios::out);
    for (int i = 0; i <= positionNumber; ++i){
        g2o::VertexSE3Quat *curPoseVert = static_cast<g2o::VertexSE3Quat *>(optimizerMin.vertex(i));
        g2o::Vector7d poseVect = curPoseVert->estimate().toVector();
        trajectoryFile << poseVect[0] << " " << poseVect[1] << " " << poseVect[2] << " " << poseVect[6] << " "
                       << poseVect[3] << " " << poseVect[4] << " " << poseVect[5] << "\n";
    }
    trajectoryFile.close();
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

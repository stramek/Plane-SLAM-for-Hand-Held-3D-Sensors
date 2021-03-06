#include "include/utils/utils.h"

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace octomap;

namespace utils {

    void paintPixel(Mat &rgb, const Vector3f &vector, Vec3b &color) {
        for (int i = 0; i < 3; ++i) {
            rgb.at<Vec3b>((int) vector[0], (int) vector[1])[i] = (uchar) color[i];
        }
    }

    pair<int, int> getRandomPosition(const Mat &mat, int areaSize) {
        return getRandomPosition(mat.cols, mat.rows, areaSize);
    }

    pair<int, int> getRandomPosition(const int cols, const int rows, int areaSize) {
        random_device rd;
        mt19937 rng(rd());
        uniform_int_distribution<int> colUni((areaSize - 1) / 2, cols - ((areaSize - 1) / 2) - 1);
        uniform_int_distribution<int> rowUni((areaSize - 1) / 2, rows - ((areaSize - 1) / 2) - 1);
        return pair<int, int>(rowUni(rng), colUni(rng));
    }

    void createOctoMap(const string filename, float resolution) {
        remove((filename + ".ot").c_str());
        ColorOcTree octree(resolution);
        if (octree.write(filename + ".ot")) {
            cout << filename << ".ot generated with success!" << endl;
        } else {
            cout << filename << ".ot generation failed!" << endl;
        }
    }

    void updateOctoMap(const string filename, const vector<Point3D> pointCloud) {
        if (pointCloud.empty()) throw runtime_error("Passed pointcloud is empty! Did you forget to call updateCloud() method?");

        AbstractOcTree* tree = AbstractOcTree::read(filename + ".ot");
        ColorOcTree* octree = dynamic_cast<ColorOcTree*>(tree);

        for (Point3D p : pointCloud) {
            octree->updateNode(point3d((float)p.position(0), (float)p.position(1), (float)p.position(2)), true);
        }
        for (Point3D p : pointCloud) {
            octree->integrateNodeColor((float)p.position(0), (float)p.position(1), (float)p.position(2), p.red, p.green, p.blue);
        }
        octree->updateInnerOccupancy();
        if (octree->write(filename + ".ot")) {
            cout << filename << ".ot UPDATED with success!" << endl;
        } else {
            cout << filename << ".ot UPDATE failed!" << endl;
        }
    }

    void appendTrajectoryRecord(string fileName, const PosOrient &posOrient) {
        ofstream stream;
        stream.open(fileName, std::ios_base::app);
        Vector3d position = posOrient.getPosition();
        Quaterniond q  = posOrient.getQuaternion();
        stream << utils::getCurrentDate() << " " << position[0] << " " << position[1] << " " << position[2] << " " << q.w() << " " << q.x()
                       << " " << q.y() << " " << q.z() << "\n";
        stream.close();
    }

    double sTod(string s) {
        return boost::lexical_cast<double>(s);
    }

    void loadDatasetPositions(vector<PosOrient> &positions) {
        cout<<"Loading file \""<<"dataset_data.txt"<<"\"... ";
        ifstream myfile("../dataset_data.txt");

        if (myfile.good()) {
            std::string line;
            while (std::getline(myfile, line)) {
                std::vector<std::string> loadedValues;
                boost::split(loadedValues, line, boost::is_any_of(" "));

                PosOrient posOrient(Vector3d(sTod(loadedValues[1]), sTod(loadedValues[2]), sTod(loadedValues[3])),
                                    Vector4d(sTod(loadedValues[4]), sTod(loadedValues[5]), sTod(loadedValues[6]), sTod(loadedValues[7])));
                positions.push_back(posOrient);
            }
            cout<<"Success!"<<endl;
        } else {
            cout<<"Could not load file!"<<endl;
            exit(-1);
        }
    }

    string getCurrentDate() {
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];
        time (&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(buffer,sizeof(buffer),"%d-%m-%Y_%I:%M:%S",timeinfo);
        std::string str(buffer);
        return str;
    }

    void rotatePoint(Point3D &point3D, const PosOrient &posOrient) {
        rotatePoint(point3D.position, posOrient);
    }

    void rotatePoint(Vector3d &point3D, const PosOrient &posOrient) {
        auto rotMatrix = posOrient.getQuaternion().toRotationMatrix();
        auto translation = posOrient.getPosition();
        point3D = (rotMatrix * point3D) + translation;
    }

    void movePlanesToPreviousVector(vector<Plane> &planeVectorPreviousFrame, vector<Plane> &planeVectorCurrentFrame) {
        planeVectorPreviousFrame.clear();
        copy(planeVectorCurrentFrame.begin(), planeVectorCurrentFrame.end(), back_inserter(planeVectorPreviousFrame));
        planeVectorCurrentFrame.clear();
    }
}
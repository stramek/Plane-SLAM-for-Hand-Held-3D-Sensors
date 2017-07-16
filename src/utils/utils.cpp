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

    void generateOctoMap(const string filename, const vector<Point3D> pointCloud, const float resolution) {
        if (pointCloud.empty()) throw runtime_error("Passed pointcloud is empty! Did you forget to call updateCloud() method?");
        ColorOcTree tree(resolution);
        for (Point3D p : pointCloud) {
            tree.updateNode(point3d((float)p.position(0), (float)p.position(1), (float)p.position(2)), true);
        }
        for (Point3D p : pointCloud) {
            tree.integrateNodeColor((float)p.position(0), (float)p.position(1), (float)p.position(2), p.red, p.green, p.blue);
        }
        tree.updateInnerOccupancy();
        if (tree.write(filename + ".ot")) {
            cout << filename << ".ot generated with success!" << endl;
        } else {
            cout << filename << ".ot generation failed!" << endl;
        }
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

    void movePlanesToPreviousVector(vector<Plane> &planeVectorPreviousFrame, vector<Plane> &planeVectorCurrentFrame) {
        planeVectorPreviousFrame.clear();
        copy(planeVectorCurrentFrame.begin(), planeVectorCurrentFrame.end(), back_inserter(planeVectorPreviousFrame));
        planeVectorCurrentFrame.clear();
    }
}
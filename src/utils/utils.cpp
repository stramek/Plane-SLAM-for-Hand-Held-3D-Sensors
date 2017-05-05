#include "include/utils/utils.h"

namespace utils {

    void paintPixel(Mat &rgb, const Vector3f &vector, Vec3b color) {
        for (int i = 0 ; i < 3; ++i) {
            rgb.at<Vec3b>((int)vector[0], (int)vector[1])[i] = (uchar) color[i];
        }
    }

    pair<int, int> getRandomPosition(const Mat &mat, int areaSize) {
        random_device rd;
        mt19937 rng(rd());
        uniform_int_distribution<int> colUni((areaSize - 1) / 2, mat.cols - ((areaSize - 1) / 2) - 1);
        uniform_int_distribution<int> rowUni((areaSize - 1) / 2, mat.rows - ((areaSize - 1) / 2) - 1);
        return pair<int, int>(rowUni(rng), colUni(rng));
    }

    void generateOctoMap(const string filename, const vector<Point3D> pointCloud, const float resolution) {
        ColorOcTree tree(resolution);
        for (Point3D p : pointCloud) {
            tree.updateNode(point3d(p.x, p.y, p.z), true);
        }
        for (Point3D p : pointCloud) {
            tree.integrateNodeColor(p.x, p.y, p.z, p.red, p.green, p.blue);
        }
        tree.updateInnerOccupancy();
        if (tree.write(filename + ".ot")) {
            cout<<filename<<".ot generated with success!"<<endl;
        } else {
            cout<<filename<<".ot generation failed!"<<endl;
        }
    }

    void mergePlanes(vector<Plane> &planeVector){
        vector<vector<Plane>> clusteredPLanes;
        Clustering::getClusteredPlaneGroup(planeVector, clusteredPLanes);
        planeVector = Clustering::getAveragedPlanes(clusteredPLanes);

    }

    void displayClusteredPlanes(ImagePair &imagePair, vector<Plane> plane){
        vector<vector<Plane>> clusteredPLanes;
        Clustering::getClusteredPlaneGroup(plane, clusteredPLanes);
        int i = 0;
        for (auto singleCluster : clusteredPLanes){
            ++i;
            for(auto planeInCluster : singleCluster){
                putText(imagePair.getRgb(), to_string(i), Point(planeInCluster.getImageCoords().getCenterX(), planeInCluster.getImageCoords().getCenterY()),FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            }
        }
        imshow("Clustered planes", imagePair.getRgb());
    }

}
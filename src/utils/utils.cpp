#include "include/utils/utils.h"

namespace utils {

    void paintPixel(Mat &rgb, const Vector3f &vector, Vec3b color) {
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
        ColorOcTree tree(resolution);
        for (Point3D p : pointCloud) {
            tree.updateNode(point3d(p.x, p.y, p.z), true);
        }
        for (Point3D p : pointCloud) {
            tree.integrateNodeColor(p.x, p.y, p.z, p.red, p.green, p.blue);
        }
        tree.updateInnerOccupancy();
        if (tree.write(filename + ".ot")) {
            cout << filename << ".ot generated with success!" << endl;
        } else {
            cout << filename << ".ot generation failed!" << endl;
        }
    }

    /*typedef struct {
        double r;       // a fraction between 0 and 1
        double g;       // a fraction between 0 and 1
        double b;       // a fraction between 0 and 1
    } rgb;

    typedef struct {
        double h;       // angle in degrees
        double s;       // a fraction between 0 and 1
        double v;       // a fraction between 0 and 1
    } hsv;*/

    //static hsv   rgb2hsv(rgb in);
    //static rgb   hsv2rgb(hsv in);



}
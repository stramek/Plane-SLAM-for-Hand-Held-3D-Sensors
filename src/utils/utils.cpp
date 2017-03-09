#include "../../include/utils/utils.h"

namespace utils {

    void paintPixel(Mat &rgb, const Vector3f &vector, Vec3b color) {
        for (int i = 0 ; i < 3; ++i) {
            rgb.at<Vec3b>((int)vector[0], (int)vector[1])[i] = (uchar) color[i];
        }
    }

    pair<int, int> getRandomPosition(const Mat &mat, int areaSize) {
        random_device rd;
        mt19937 rng(rd());
        uniform_int_distribution<int> colUni((areaSize - 1) / 2, mat.cols - ((areaSize - 1) / 2));
        uniform_int_distribution<int> rowUni((areaSize - 1) / 2, mat.rows - ((areaSize - 1) / 2));
        return pair<int, int>(rowUni(rng), colUni(rng));
    }

}
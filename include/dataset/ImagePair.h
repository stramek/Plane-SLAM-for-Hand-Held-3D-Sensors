//
// Created by stramek on 26.02.17.
//

#ifndef PROJEKTMAGISTERSKI_IMAGEPAIR_H
#define PROJEKTMAGISTERSKI_IMAGEPAIR_H

#include <opencv2/opencv.hpp>

using namespace cv;

class ImagePair {
public:

    ImagePair();

    ImagePair(const Mat &rgb, const Mat &depth);

    const Mat &getRgb() const;

    const Mat &getDepth() const;

    void setRgb(const Mat &rgb);

    void setDepth(const Mat &depth);

    int getDepthAt(int x, int y);

private:
    Mat rgb;
    Mat depth;
};


#endif //PROJEKTMAGISTERSKI_IMAGEPAIR_H

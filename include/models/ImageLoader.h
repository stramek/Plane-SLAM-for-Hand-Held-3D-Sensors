//
// Created by stramek on 26.02.17.
//

#ifndef PROJEKTMAGISTERSKI_IMAGELOADER_H
#define PROJEKTMAGISTERSKI_IMAGELOADER_H

#include <opencv2/opencv.hpp>
#include "ImagePair.h"

using namespace cv;
using namespace std;

class ImageLoader {
public:

    ImageLoader(int numberOfPhotos);

    ImagePair getNextPair();

private:

    int numberOfPhotos;
    int currentPhoto;

    ImagePair loadNextImage();
};


#endif //PROJEKTMAGISTERSKI_IMAGELOADER_H
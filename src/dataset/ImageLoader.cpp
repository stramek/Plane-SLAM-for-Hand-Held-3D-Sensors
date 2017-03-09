//
// Created by stramek on 26.02.17.
//

#include "include/dataset/ImageLoader.h"

ImageLoader::ImageLoader(int numberOfPhotos) : numberOfPhotos(numberOfPhotos - 1) {
    currentPhoto = 0;
}

ImagePair ImageLoader::getNextPair() {
    return loadNextImage();
}

ImagePair ImageLoader::loadNextImage() {
    if (currentPhoto > numberOfPhotos) currentPhoto = 0;

    ImagePair imagePair;
    imagePair.setRgb(imread("../dataset_photos//rgb//" + to_string(currentPhoto) + ".png"));
    imagePair.setDepth(imread("../dataset_photos//depth//" + to_string(currentPhoto) + ".png", CV_LOAD_IMAGE_ANYDEPTH));
    ++currentPhoto;

    return imagePair;
}


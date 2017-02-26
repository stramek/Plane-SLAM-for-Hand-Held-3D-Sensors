//
// Created by stramek on 26.02.17.
//

#include "include/dataset/ImagePair.h"

ImagePair::ImagePair() {}

ImagePair::ImagePair(const Mat &rgb, const Mat &depth) : rgb(rgb), depth(depth) {}

const Mat &ImagePair::getRgb() const {
    return rgb;
}

const Mat &ImagePair::getDepth() const {
    return depth;
}

void ImagePair::setRgb(const Mat &rgb) {
    ImagePair::rgb = rgb;
}

void ImagePair::setDepth(const Mat &depth) {
    ImagePair::depth = depth;
}
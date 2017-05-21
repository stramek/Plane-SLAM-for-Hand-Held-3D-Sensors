//

#include "include/models/HSVColor.h"

//
//  HSVColor.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 01.05.2017.
//
HSVColor::HSVColor() {
}


/**
 * Calculates hue, saturation and value of passed Mat object
 * @param colorImage
 * RGB Mat which should by analyzed
 */
HSVColor::HSVColor(const Mat &colorImage) {
    Mat hsvImage;
    cvtColor(colorImage, hsvImage, CV_BGR2HSV);

    long numberOfPixels = colorImage.rows * colorImage.cols;
    long saturationSum = 0;
    long hueSum = 0;
    long valueSum = 0;

    for (int r = 0; r < colorImage.rows; ++r) {
        const Pixel *ptr = colorImage.ptr<Pixel>(r, 0);
        const Pixel *ptr_end = ptr + colorImage.cols;
        for (; ptr != ptr_end; ++ptr) {
            hueSum += ptr->val[0];
            saturationSum += ptr->val[1];
            valueSum += ptr->val[2];
        }
    }

    hue = (uint8_t) (hueSum / numberOfPixels);
    saturation = (uint8_t) (saturationSum / numberOfPixels);
    value = (uint8_t) (valueSum / numberOfPixels);
}

HSVColor::HSVColor(uint8_t hue, uint8_t saturation, uint8_t value) {
    this->hue = hue;
    this->saturation = saturation;
    this->value = value;
}

/**
 * @return 0 - 180 range hue of HSV color
 */
uint8_t HSVColor::getHue() const {
    return hue;
}


/**
 * @return 0 - 255 range saturation of HSV color
 */
uint8_t HSVColor::getSaturation() const {
    return saturation;
}

/**
 * @return 0 - 255 range value of HSV color
 */
uint8_t HSVColor::getValue() const {
    return value;
}

void HSVColor::setHue(uint8_t hue) {
    HSVColor::hue = hue;
}

Vec3b HSVColor::convertBgr2Hsv(Vec3b src) {
    Mat srcMat(1, 1, CV_8UC3);
    *srcMat.ptr<Vec3b>(0) = src;
    Mat resMat;
    cvtColor(srcMat, resMat, CV_BGR2HSV);
    return *resMat.ptr<Vec3b>(0);
}

HSVColor::HSVColor(const vector<Point3D> &points) {

    long saturationSum = 0;
    long hueSum = 0;
    long valueSum = 0;

    for (Point3D point : points) {
        Vec3b hsv = convertBgr2Hsv(Vec3b(point.blue, point.green, point.red));
        hueSum += hsv[0];
        saturationSum += hsv[1];
        valueSum += hsv[2];
    }

    hue = (uint8_t) (hueSum / points.size());
    saturation = (uint8_t) (saturationSum / points.size());
    value = (uint8_t) (valueSum / points.size());
}



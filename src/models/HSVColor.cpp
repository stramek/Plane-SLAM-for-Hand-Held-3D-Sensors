//

#include "../../include/models/HSVColor.h"

//
//  HSVColor.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 01.05.2017.
//
HSVColor::HSVColor() {
}

HSVColor::HSVColor(const Mat &colorImage) {
    Mat hsvImage;
    cvtColor(colorImage, hsvImage, CV_BGR2HSV);

    long numberOfPixels = colorImage.rows * colorImage.cols;
    long saturationSum = 0;
    long hueSum = 0;
    long valueSum = 0;

    for (int r = 0; r < colorImage.rows; ++r) {
        const Pixel* ptr = colorImage.ptr<Pixel>(r, 0);
        const Pixel* ptr_end = ptr + colorImage.cols;
        for (; ptr != ptr_end; ++ptr) {
            hueSum += ptr->val[0];
            saturationSum += ptr->val[1];
            valueSum += ptr->val[2];
        }
        //delete(ptr);
        //delete(ptr_end);
    }

    hue = (uint8_t) (hueSum / numberOfPixels);
    saturation = (uint8_t) (saturationSum / numberOfPixels);
    value = (uint8_t) (valueSum / numberOfPixels);
}

uint8_t HSVColor::getHue() const {
    return hue;
}

uint8_t HSVColor::getSaturation() const {
    return saturation;
}

uint8_t HSVColor::getValue() const {
    return value;
}

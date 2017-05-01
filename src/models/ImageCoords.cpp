//

#include "../../include/models/ImageCoords.h"

//
//  ImageCoords.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 01.05.2017.
//

ImageCoords::ImageCoords(const pair<int, int> &position, const int areaSize) {
    upLeftX = position.second - (areaSize - 1) / 2;
    upLeftY = position.first - (areaSize - 1) / 2;
    downRightX = position.second + (areaSize - 1) / 2;
    downRightY = position.first + (areaSize - 1) / 2;
    this->areaSize = areaSize;
}

int ImageCoords::getUpLeftX() const {
    return upLeftX;
}

int ImageCoords::getUpLeftY() const {
    return upLeftY;
}

int ImageCoords::getDownRightX() const {
    return downRightX;
}

int ImageCoords::getDownRightY() const {
    return downRightY;
}

int ImageCoords::getAreaSize() const {
    return areaSize;
}

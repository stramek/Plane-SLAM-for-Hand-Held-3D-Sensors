//

#include "../../include/utils/ImageCoords.h"

//
//  ImageCoords.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 01.05.2017.
//

/**
 * Calculates up left and down right corners
 * @param position
 * Center of area (x, y)
 * @param areaSize
 * Area size
 */
ImageCoords::ImageCoords(const pair<int, int> &position, const int areaSize) {
    upLeftX = position.second - (areaSize - 1) / 2;
    upLeftY = position.first - (areaSize - 1) / 2;
    downRightX = position.second + (areaSize - 1) / 2;
    downRightY = position.first + (areaSize - 1) / 2;
    this->areaSize = areaSize;
}

/**
 * @return
 * Upper left X coordinate of calculated area
 */
int ImageCoords::getUpLeftX() const {
    return upLeftX;
}

/**
 * @return
 * Upper left Y coordinate of calculated area
 */
int ImageCoords::getUpLeftY() const {
    return upLeftY;
}

/**
 * @return
 * Upper right X coordinate of calculated area
 */
int ImageCoords::getDownRightX() const {
    return downRightX;
}

/**
 * @return
 * Upper right Y coordinate of calculated area
 */
int ImageCoords::getDownRightY() const {
    return downRightY;
}

/**
 * @return
 * Passed area size
 */
int ImageCoords::getAreaSize() const {
    return areaSize;
}

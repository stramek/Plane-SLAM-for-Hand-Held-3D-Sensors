//
//  Plane.hpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 11.02.2017.
//
//

#ifndef ImageCoords_h
#define ImageCoords_h
#ifdef __GNUC__
// Avoid tons of warnings with root code
#pragma GCC system_header
#endif

#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

class ImageCoords {
public:
    ImageCoords();

    ImageCoords(const pair<int, int> &position, const int areaSize);
    int getUpLeftX() const;
    int getUpLeftY() const;
    int getDownRightX() const;
    int getDownRightY() const;
    int getCenterX() const;
    int getCenterY() const;
    int getAreaSize() const;
private:
    int upLeftX = -1;
    int upLeftY = -1;
    int downRightX = -1;
    int downRightY = -1;
    int centerX = -1;
    int centerY = -1;
    int areaSize = -1;
};

#endif

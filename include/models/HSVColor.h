//
//  Plane.hpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 11.02.2017.
//
//

#ifndef HSVColor_h
#define HSVColor_h
#ifdef __GNUC__
// Avoid tons of warnings with root code
#pragma GCC system_header
#endif

#include <opencv2/opencv.hpp>
#include "include/models/Point3D.h"
using namespace cv;
using namespace std;

typedef Vec<uchar, 3> Pixel;

class HSVColor {
public:
    HSVColor();
    HSVColor(const vector<Point3D> &points);
    HSVColor(const Mat& colorImage);
    HSVColor(uint8_t hue, uint8_t saturation, uint8_t value);
    uint8_t getHue() const;
    uint8_t getSaturation() const;
    uint8_t getValue() const;
    void setHue(uint8_t hue);
private:
    uint8_t hue = 0;
    uint8_t saturation = 0;
    uint8_t value = 0;
    Vec3b convertBgr2Hsv(Vec3b src);
};

#endif

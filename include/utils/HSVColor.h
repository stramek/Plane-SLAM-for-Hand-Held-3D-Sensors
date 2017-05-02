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
using namespace cv;

typedef Vec<uchar, 3> Pixel;

class HSVColor {
public:
    HSVColor();

    HSVColor(const Mat& colorImage);
    uint8_t getHue() const;
    uint8_t getSaturation() const;
    uint8_t getValue() const;
private:
    uint8_t hue = 0;
    uint8_t saturation = 0;
    uint8_t value = 0;
};

#endif

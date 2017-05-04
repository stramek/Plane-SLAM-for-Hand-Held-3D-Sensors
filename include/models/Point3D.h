//
//  Plane.hpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 11.02.2017.
//
//

#ifndef Point3D_h
#define Point3D_h
#ifdef __GNUC__
// Avoid tons of warnings with root code
#pragma GCC system_header
#endif

struct Point3D {
    Point3D() {}
    Point3D(float x, float y, float z, unsigned char red, unsigned char green, unsigned char blue) : x(x), y(y), z(z),
                                                                                                     red(red),
                                                                                                     green(green),
                                                                                                     blue(blue) {}
    float x;
    float y;
    float z;
    unsigned char red;
    unsigned char green;
    unsigned char blue;
};

#endif /* Point3D_h */

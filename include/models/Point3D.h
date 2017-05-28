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

#include <Eigen/Dense>

struct Point3D {
    Point3D() {}
    Point3D(double x, double y, double z, unsigned char red, unsigned char green, unsigned char blue) {
        position(0) = x;
        position(1) = y;
        position(2) = z;
        this->red = red;
        this->green = green;
        this->blue = blue;
    }

    Eigen::Vector3d position;
    unsigned char red;
    unsigned char green;
    unsigned char blue;
};

#endif /* Point3D_h */

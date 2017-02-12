//
//  libraries_include.h
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 14.12.2016.
//
//

#ifndef libraries_include_h
#define libraries_include_h

#ifdef __GNUC__
// Avoid tons of warnings with root code
#pragma GCC system_header
#endif

#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <cstdlib>

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <Eigen/Dense>

#endif /* libraries_include_h */

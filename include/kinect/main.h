//
//  main.h
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 14.12.2016.
//
//

#ifndef main_h
#define main_h

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
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <Eigen/Dense>
#include <rxcpp/rx.hpp>
#include "include/utils/Qvisualizer.h"
#include <QApplication>
#include <include/utils/utils.h>
#include "include/utils/planeUtils.h"
#include "include/algorithms/PcaPlaneDetector.h"
#include "include/algorithms/RansacPlaneDetector.h"
#include "include/models/PlaneFiller.h"
#include "KinectModule.h"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace libfreenect2;

#endif /* main_h */

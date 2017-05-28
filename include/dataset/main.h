//
//  Main.h
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 11.02.2017.
//
//

#ifndef Main_h
#define Main_h
#ifdef __GNUC__
// Avoid tons of warnings with root code
#pragma GCC system_header
#endif

#include <iostream>
#include <QGLViewer/qglviewer.h>
#include <qapplication.h>
#include <GL/glut.h>
#include "include/utils/Qvisualizer.h"
#include "include/utils/constants.h"
#include "include/models/ImageCoords.h"
#include <opencv2/opencv.hpp>
#include "include/models/ImageLoader.h"
#include <rxcpp/rx.hpp>
#include <include/utils/utils.h>
#include <include/algorithms/PcaPlaneDetector.h>
#include "include/utils/planeUtils.h"
#include "include/models/PlaneFiller.h"
#include "include/algorithms/RansacPlaneDetector.h"

namespace Rx {
    using namespace rxcpp;
    using namespace rxcpp::sources;
    using namespace rxcpp::operators;
    using namespace rxcpp::util;
}

using namespace Rx;
using namespace std;
using namespace cv;

#endif

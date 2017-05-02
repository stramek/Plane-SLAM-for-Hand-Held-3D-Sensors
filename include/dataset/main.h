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
#include "Qvisualizer.h"
#include "../utils/constants.h"
#include "include/utils/ImageCoords.h"
#include <opencv2/opencv.hpp>
#include "ImageLoader.h"
#include <rxcpp/rx.hpp>
#include <include/utils/utils.h>
#include <include/dataset/PlanePca.h>

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

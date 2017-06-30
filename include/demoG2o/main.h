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
#include <include/utils/utils.h>
#include <include/algorithms/PcaPlaneDetector.h>
#include "include/utils/planeUtils.h"
#include "include/models/PlaneFiller.h"
#include "include/algorithms/RansacPlaneDetector.h"

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

using namespace g2o;
using namespace std;

#endif

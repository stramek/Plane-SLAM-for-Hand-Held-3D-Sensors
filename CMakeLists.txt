cmake_minimum_required(VERSION 3.5.1)

Project (ProjektMagisterski)

SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
SET(CMAKE_BUILD_TYPE Release)
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}")
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${freenect2_INCLUDE_DIRS})

#QGLViewer library
FIND_PACKAGE(OpenGL)
include_directories(${OPENGL_INCLUDE_DIR})
link_directories(${OPENGL_LIBRARY})

#QGLViewer library
FIND_PACKAGE(QGLViewer)
include_directories(${QGLVIEWER_INCLUDE_DIR})
include_directories(${QT_INCLUDES})
link_directories(${QGLVIEWER_LIBRARY})

#CSparse library
FIND_PACKAGE(CSparse)
include_directories(${CSPARSE_INCLUDE_DIR})
link_directories(${CSPARSE_LIBRARY})

#G2O library
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})
link_directories(${G2O_LIBRARY_DIRS})

#octomap library
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

################ MODELS ################

file(GLOB MODEL_SOURCES
   "${CMAKE_SOURCE_DIR}/src/models/*.cpp"
)
file(GLOB MODEL_HEADERS
   "${CMAKE_SOURCE_DIR}/include/models/*.h"
)
ADD_LIBRARY(Models STATIC ${MODEL_SOURCES} ${MODEL_HEADERS})
INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/models/)

#########################################

################ UTILS ################

file(GLOB UTILS_SOURCES
   "${CMAKE_SOURCE_DIR}/src/utils/*.cpp"
)
file(GLOB UTILS_HEADERS
   "${CMAKE_SOURCE_DIR}/include/utils/*.h"
)
ADD_LIBRARY(Utils STATIC ${UTILS_SOURCES} ${UTILS_HEADERS})
TARGET_LINK_LIBRARIES(Utils Models Algorithms)
INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/utils/)

#########################################

########### PLANE ALGORITHMS ############

file(GLOB ALGORITHMS_SOURCES
        "${CMAKE_SOURCE_DIR}/src/algorithms/*.cpp"
)
file(GLOB ALGoRITHMS_HEADERS
        "${CMAKE_SOURCE_DIR}/include/algorithms/*.h"
)
ADD_LIBRARY(Algorithms STATIC ${ALGORITHMS_SOURCES} ${ALGORITHMS_HEADERS})
TARGET_LINK_LIBRARIES(Algorithms Utils Models)
INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/algorithms/)

#########################################

############### PLANE G2O ###############

file(GLOB PLANEG2O_SOURCES
        "${CMAKE_SOURCE_DIR}/src/planeG2O/*.cpp"
)
file(GLOB PLANEG2O_HEADERS
        "${CMAKE_SOURCE_DIR}/include/planeG2O/*.h"
)
ADD_LIBRARY(PlaneG2O STATIC ${PLANEG2O_SOURCES} ${PLANEG2O_HEADERS})
TARGET_LINK_LIBRARIES(PlaneG2O Utils Models g2o_types_slam2d g2o_types_slam3d g2o_csparse_extension g2o_stuff g2o_core g2o_solver_csparse
        ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${QT_LIBRARIES} QtCore QtXml QtOpenGL QtGui ${OPENGL_LIBRARY} glut)
INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/planeG2O/)

#########################################

################ Demo G2O ################

file(GLOB DEMOG2O_SOURCES
        "${CMAKE_SOURCE_DIR}/src/demoG2o/*.cpp"
)
file(GLOB DEMOG2O_HEADERS
        "${CMAKE_SOURCE_DIR}/include/demoG2o/*.h"
)

ADD_EXECUTABLE(DemoG2o ${DEMOG2O_SOURCES} ${DEMOG2O_HEADERS})
TARGET_LINK_LIBRARIES(DemoG2o Models Utils Algorithms PlaneG2O ${OpenCV_LIBRARIES} ${freenect2_LIBRARIES}
        ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${QT_LIBRARIES} csparse cholmod QtCore QtXml QtOpenGL QtGui ${OPENGL_LIBRARY} glut
        g2o_types_sba g2o_types_slam2d g2o_types_slam3d g2o_csparse_extension g2o_stuff g2o_core g2o_solver_csparse)

#########################################

################ Kinect ################

file(GLOB MAIN_SOURCES
  "${CMAKE_SOURCE_DIR}/src/kinect/*.cpp"
)
file(GLOB MAIN_HEADERS
  "${CMAKE_SOURCE_DIR}/include/kinect/*.h"
)

ADD_EXECUTABLE(Kinect ${MAIN_SOURCES} ${MAIN_HEADERS})
TARGET_LINK_LIBRARIES(Kinect Models Utils Algorithms PlaneG2O ${OpenCV_LIBRARIES} ${freenect2_LIBRARIES}
    ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${QT_LIBRARIES} QtCore QtXml QtOpenGL QtGui ${OPENGL_LIBRARY} glut)

#########################################

################# Dataset ###############

file(GLOB DATASET_SOURCES
  "${CMAKE_SOURCE_DIR}/src/dataset/*.cpp"
 )
 file(GLOB DATASET_HEADERS
  "${CMAKE_SOURCE_DIR}/include/dataset/*.h"
)
ADD_EXECUTABLE(Dataset ${DATASET_SOURCES} ${DATASET_HEADERS})
TARGET_LINK_LIBRARIES(Dataset Models Utils Algorithms PlaneG2O ${OpenCV_LIBRARIES} ${freenect2_LIBRARIES}
    ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${QT_LIBRARIES} QtCore QtXml QtOpenGL QtGui ${OPENGL_LIBRARY} glut)

#########################################
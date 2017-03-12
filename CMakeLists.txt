cmake_minimum_required(VERSION 3.5.1)

Project (ProjektMagisterski)

SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
SET(CMAKE_BUILD_TYPE Debug)
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}")
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(RxCpp REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${freenect2_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${RxCpp_INCLUDE_DIRS})

#QGLViewer library
FIND_PACKAGE(OpenGL)
include_directories(${OPENGL_INCLUDE_DIR})
link_directories(${OPENGL_LIBRARY})

#QGLViewer library
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
FIND_PACKAGE(QGLViewer)
include_directories(${QGLVIEWER_INCLUDE_DIR})
include_directories(${QT_INCLUDES})
link_directories(${QGLVIEWER_LIBRARY})

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
INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/utils/)

#########################################

############### CLUSTERING ##############

file(GLOB CLUSTERING_SOURCES
        "${CMAKE_SOURCE_DIR}/src/clustering/*.cpp"
)
file(GLOB CLUSTERING_HEADERS
        "${CMAKE_SOURCE_DIR}/include/clustering/*.h"
)
ADD_LIBRARY(Clustering STATIC ${CLUSTERING_SOURCES} ${CLUSTERING_HEADERS})
INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/clustering/)

#########################################

################ Kinect ################

file(GLOB MAIN_SOURCES
  "${CMAKE_SOURCE_DIR}/src/kinect/*.cpp"
)
file(GLOB MAIN_HEADERS
  "${CMAKE_SOURCE_DIR}/include/kinect/*.h"
)

ADD_EXECUTABLE(Kinect ${MAIN_SOURCES} ${MAIN_HEADERS})
TARGET_LINK_LIBRARIES(Kinect Models Utils ${OpenCV_LIBRARIES} ${freenect2_LIBRARIES}
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
TARGET_LINK_LIBRARIES(Dataset Models Utils Clustering ${OpenCV_LIBRARIES} ${freenect2_LIBRARIES}
    ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${QT_LIBRARIES} QtCore QtXml QtOpenGL QtGui ${OPENGL_LIBRARY} glut)

#########################################
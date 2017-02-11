//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "../include/main.h"

enum Processor { cl, gl, cpu };

bool protonect_shutdown = false; // Whether the running application should shut down.
static const int DEPTH_PROCESSOR = Processor::cl;

void sigint_handler(int s) {
    protonect_shutdown = true;
}

libfreenect2::Freenect2 freenect2;
libfreenect2::Freenect2Device *dev = nullptr;
libfreenect2::PacketPipeline *pipeline = nullptr;

void quitIfDeviceNotConnected() {
    if(freenect2.enumerateDevices() == 0) {
        throw std::runtime_error("Didn't find any kinect.");
    }
}

void setDepthProcessor() {
    switch (DEPTH_PROCESSOR) {
        case Processor::cpu:
            if(!pipeline) {
                pipeline = new libfreenect2::CpuPacketPipeline();
            }
            break;
        case Processor::gl:
             #ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
                if(!pipeline) {
                    pipeline = new libfreenect2::OpenGLPacketPipeline();
                }
             #else
                throw std::runtime_error("OpenGL pipeline is not supported!");
             #endif
            break;
        case Processor::cl:
             #ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
                if(!pipeline) {
                    pipeline = new libfreenect2::OpenCLPacketPipeline();
                }
             #else
                throw std::runtime_error("OpenCL pipeline is not supported!");
             #endif
            break;
        default:
            throw std::runtime_error("Wrong depth processor!");
            break;
    }
}

void openDevice() {
    string serial = freenect2.getDefaultDeviceSerialNumber();
    dev = pipeline ? freenect2.openDevice(serial, pipeline) : freenect2.openDevice(serial);
    
    if(dev == 0) {
        throw std::runtime_error("Failure opening device!");
    }
}


int main()
{
  Vector3d v(1,2,3);
  Vector3d w(0,1,2);
  cout << "Dot product: " << v.dot(w) << endl;
  double dp = v.adjoint()*w; // automatic conversion of the inner product to a scalar
  cout << "Dot product via a matrix product: " << dp << endl;
  cout << "Cross product:\n" << v.cross(w) << endl;
}

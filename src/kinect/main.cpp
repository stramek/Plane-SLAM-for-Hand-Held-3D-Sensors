//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "../../include/kinect/main.h"

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

void rxCppTestCode() {
    cout<<"before RX";
    
    array<int, 8> a = {{1, 2, 3, 4, 5, 6, 7, 8}};
    
    auto values = observable<>::iterate(a, observe_on_event_loop());
    values
        .filter([](int value){
        return value % 2;
        })
        .map([](int v) {
        return v + 5;
        })
        .map([](int v) -> int {
        return v + 10;
        })
        .as_dynamic()
        .subscribe([](int v) {
            printf("\nOnNext: %d", v);
        },
        []() {
            printf("\nOnCompleted");
        });
    cout<<"\nafter RX";
    
    usleep(3000 * 1000);
    cout<<"\nafter sleep\n";
}

int main() {
    //rxCppTestCode();
    quitIfDeviceNotConnected();
    setDepthProcessor();
    openDevice();

    signal(SIGINT, sigint_handler);

    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth | libfreenect2::Frame::Ir);

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();
    
    libfreenect2::FrameMap frames;
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

    Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;
    
    while(!protonect_shutdown) {
        
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

        resize(rgbmat, rgbmat, Size(), 0.3, 0.3, INTER_LINEAR);

        cv::imshow("rgb", rgbmat);
        cv::imshow("ir", irmat / 4500.0f);
        cv::imshow("depth", depthmat / 4500.0f);

        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

        cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

        resize(rgbd2, rgbd2, Size(), 0.3, 0.3, INTER_LINEAR);

        cv::imshow("undistorted", depthmatUndistorted / 4500.0f);
        cv::imshow("registered", rgbd);
        cv::imshow("depth2RGB", rgbd2 / 4500.0f);

        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        listener.release(frames);
    }

    dev->stop();
    dev->close();

    delete registration;
    return 0;
}

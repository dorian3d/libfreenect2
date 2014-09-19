/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include <iostream>
#include <signal.h>

#include <libfreenect2/opengl.h>

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>

#include <libfreenect2/depth_registration.h>

bool shutdown = false;

void sigint_handler(int s)
{
  shutdown = true;
}


int main(int argc, char *argv[])
{
  std::string program_path(argv[0]);
  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";

  if(executable_name_idx != std::string::npos)
  {
    binpath = program_path.substr(0, executable_name_idx);
  }

  glfwInit();

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = freenect2.openDefaultDevice();

  if(dev == 0)
  {
    std::cout << "no device connected or failure opening the default one!" << std::endl;
    return -1;
  }

  signal(SIGINT,sigint_handler);
  shutdown = false;

  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  // align depth and rgb image
  DepthRegistration *depthReg = NULL;
  bool bflag = false;

  while(!shutdown)
  {
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb_frame = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir_frame = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth_frame = frames[libfreenect2::Frame::Depth];

    // change the size of rgb image
    cv::Mat rgb = cv::Mat(rgb_frame->height, rgb_frame->width, CV_8UC3, rgb_frame->data);
    cv::Mat ir = cv::Mat(ir_frame->height, ir_frame->width, CV_32FC1, ir_frame->data) / 20000.0f;
    cv::Mat depth = cv::Mat(depth_frame->height, depth_frame->width, CV_32FC1, depth_frame->data) / 4500.0f;
    cv::Mat scaled = cv::Mat(depth_frame->height, depth_frame->width, CV_32FC1);

    // do depth reg
    if(depth.rows != rgb.rows || depth.cols != rgb.cols)
    {
      if(bflag == false)
      {
        depthReg = DepthRegistration::New(cv::Size(rgb.cols, rgb.rows),
                                          cv::Size(depth.cols, depth.rows),
                                          cv::Size(depth.cols, depth.rows),
                                          0.5f, 20.0f, 0.015f, DepthRegistration::CPU);

        cv::Mat cameraMatrixColor, cameraMatrixDepth;
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
        depthReg->ReadDefaultCameraInfo(cameraMatrixColor, cameraMatrixDepth);

        depthReg->init(cameraMatrixColor, cameraMatrixDepth,
                       cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(1, 3, CV_64F),
                       cv::Mat::zeros(depth.rows, depth.cols, CV_32F),
                       cv::Mat::zeros(depth.rows, depth.cols, CV_32F));

        bflag=true;
      }

      depthReg->depthToRGBResolution(depth, scaled);
    }
    else
    {
      scaled = depth;
    }

    cv::Size size(640,480);//the dst image size,e.g.100x100
    resize(rgb,rgb,size);//resize image
    resize(ir,ir,size);//resize image
    resize(depth,depth,size);//resize image
    resize(scaled,scaled,size);//resize image

    cv::imshow("rgb", rgb);
    //    cv::imshow("ir", ir);
    cv::imshow("depth", scaled);

    int key = cv::waitKey(1);
    shutdown = shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

    listener.release(frames);
    //libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));
  }

  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
  dev->stop();
  dev->close();

  delete depthReg;

  return 0;
}

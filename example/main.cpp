/*
    Copyright (C) 2006-2018 Koichi Sato
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
    You should have received a copy of the GNU Lesser General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301  USA
 */
 /**
 * @author Koichi Sato (koichisato@gmail.com)
 * @version 1.0.0
 * @file main program for sample using OpenCV
 */


#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

#include "OpenCVWP.h"
#include "blob.h"

int main() {
  cv::Mat im=cv::imread("LennaBinary.png");

  auto start = std::chrono::high_resolution_clock::now();

  cv::Mat ime(im.rows,im.cols,im.type());
  cv::erode(im,ime, cv::Mat(), cv::Point(-1,-1), 1);

  auto lap = std::chrono::high_resolution_clock::now();

  SegmentsCV<std::vector> segs(im);
  segs.showInfo();
  SegmentsCV<std::vector> segs_erode = segs.erode();

  Blobs<std::vector,StatBasic> blobs(segs_erode);

  auto finish = std::chrono::high_resolution_clock::now();
  std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(lap-start).count() << "ns\n";
  std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(finish-lap).count() << "ns\n";


  const unsigned char colors[]={
    0xff,0,0xff,
    0xff,0,0,
    0,0xff,0,
    0,0,0xff,
    0xff,0xff,0,
    0,0xff,0xff
  };

  cv::Mat im2=segs_erode.convertToImage( im.elemSize() , colors, 6);

  cv::imshow("org",im);
  cv::imshow("erode(OpenCV)",ime);
  cv::imshow("erode(label)",im2);
  cv::waitKey(0);
  return 0;
}

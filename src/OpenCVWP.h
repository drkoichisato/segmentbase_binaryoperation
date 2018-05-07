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

#ifndef OPENCVWP_H
#define OPENCVWP_H

#include "Segment.h"

/**
*@class OpenCV Wrapper mainly for input/output Mat object
*/
template< template<class T, class Allocator=std::allocator<T> > class Container>
class SegmentsCV : public Segments<Container> {
public:
  SegmentsCV(){
    Segments<Container>::Segments();
  }
  SegmentsCV(const Segments<Container> &a):Segments<Container>::Segments(a) {}
  SegmentsCV(const SegmentsCV<Container> &a):Segments<Container>::Segments(a) {}
  SegmentsCV(const cv::Mat &im) {
    init(im,0x01,0);
  }
  void operator=(Segments<Container> &a) {
    Segments<Container>::operator=(a);
  }
  void init(const cv::Mat &mat,const unsigned char BIT,const int channel) {
    Segments<Container>::init(mat.ptr()+channel,mat.cols,mat.rows,mat.elemSize(),mat.step,BIT);
  }
  void init(const cv::Mat &mat,const unsigned char BIT) {
    init(mat,BIT,0);
  }
  void init(const cv::Mat &mat) {
    init(mat,0x01,0);
  }

  cv::Mat convertToImage(const int channel) {
    cv::Mat img(this->imageHeight(),this->imageWidth(),CV_MAKETYPE(CV_8U,channel));
    Segments<Container>::convertToImage(img.ptr(), channel, channel * this->imageWidth() );
    return img;
  }

  cv::Mat convertToImage(const int channel,const unsigned char *colors,const int Ncolors) {
    cv::Mat img(this->imageHeight(),this->imageWidth(),CV_MAKETYPE(CV_8U,channel));
    Segments<Container>::convertToImage(img.ptr(), channel, channel * this->imageWidth(),colors,Ncolors );
    return img;
  }
};




#endif

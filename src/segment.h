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
 * @file segment operation
 */

#ifndef SEGMENT_H
#define SEGMENT_H

#include <memory>
/**
* @class Segment
* @brief Segment class consists of 4-short
*/
class Segment {
public:
  /** L left value */
  short L;
  /** R right value */
  short R;
  /** y y value */
  short y;
  /** id used for labeling */
  unsigned short id;
public:
  Segment(short _L,short _R,short _y,short _id):L(_L),R(_R),y(_y),id(_id){}
  Segment():L(-1),R(-1),y(-1),id(0){}

  /**
  @return error Segment
  */
  static Segment error() {
    return Segment(-1,-1,-1,0);
  }
  bool isIllegal() {
    if( L < 0 )
      return false;
    if( R < 0 )
      return false;
    if( y < 0 )
      return false;
    if( L > R )
      return false;
    return true;
  }

  Segment and_withoutCheck(const Segment &a) {
    Segment ret(L,R,y,0);
    if( ret.L < a.L )
      ret.L = a.L;
    if( ret.R > a.R )
      ret.R = a.R;
    return ret;
  }

  Segment or_withoutCheck(const Segment &a) {
    Segment ret(L,R,y,0);
    if( ret.L > a.L )
      ret.L = a.L;
    if( ret.R < a.R )
      ret.R = a.R;
    return ret;
  }
  Segment operator&(const Segment &a) {
    if( y != a.y )
      return Segment::error();
    return and_withoutCheck(a);
  }
  Segment operator|(const Segment &a) {
    if( isIntersect(a) )
      return or_withoutCheck(a);
    return Segment::error();
  }
  bool isIntersect(const Segment &a) {
    if( y != a.y )
      return false;
    if( (L > a.R)||(R < a.L) )
      return false;
    return true;
  }
  bool isHorizontallyIntersect(const Segment &a) {
    if( (L > a.R)||(R < a.L) )
      return false;
    return true;
  }
  bool isConnecting4(const Segment &a) {
    if( (y - a.y > 1 )||( a.y-y >1 ) )
      return false;
    if( L > a.R )
      return false;
    if( R < a.L )
      return false;
    return true;
  }
  bool isConnecting8(const Segment &a) {
    if( (y - a.y > 1 )||( a.y-y >1 ) )
      return false;
    if( L+1 > a.R )
      return false;
    if( R+1 < a.L )
      return false;
    return true;
  }

  int findRootLabel(int id,std::unordered_map<int,int> *label) {
    while( (*label)[id] != id ) {
      id=(*label)[id];
    }
    return id;
  }

  bool connect(const Segment &a,std::unordered_map<int,int> *label) {
    if( id == 0 ) {
      id=findRootLabel(a.id,label);
      return true;
    }
    int id1=findRootLabel(id,label);
    int id2=findRootLabel(a.id,label);
    if( id1 < id2 ) {
      (*label)[id2]=id1;
      return false;
    }
    if( id1 > id2 ) {
      (*label)[id1]=id2;
      return false;
    }
    return true;
  }


};


/**
* @class Segments container class for Segment
* specify Container (std::vector will be general use)
*
*/
template< template<class T, class Allocator=std::allocator<T> > class Container> class Segments {
protected:
  std::shared_ptr<Container<Segment> > segs;
  std::shared_ptr<std::vector<short> > index;
  unsigned short w;
  unsigned short h;

protected:
  void append(const Segment &seg) {
    segs->push_back(seg);
  }

  void and_line(Container<Segment> *segout,Segment *sega,Segment *sega_end,Segment *segb,Segment *segb_end) {
    while( (sega!=sega_end)&&(segb!=segb_end) ) {
      if( sega->isHorizontallyIntersect(*segb) )
      {
        Segment seg=sega->and_withoutCheck(*segb);
        segout->push_back( seg );
      }
      if(sega->R < segb->R)
        sega++;
      else
        segb++;
    }
  }
  void and_line_shrink(Container<Segment> *segout,Segment *sega,Segment *sega_end,Segment *segb,Segment *segb_end) {
    while( (sega!=sega_end)&&(segb!=segb_end) ) {
      if( sega->isHorizontallyIntersect(*segb) )
      {
        Segment seg=sega->and_withoutCheck(*segb);
        seg.L++;
        seg.R--;
        if( seg.L < seg.R )
        {
          segout->push_back( seg );
        }
      }
      if(sega->R < segb->R)
        sega++;
      else
        segb++;
    }
  }
  void or_line(Container<Segment> *segout,Segment *sega,Segment *sega_end,Segment *segb,Segment *segb_end) {
    Segment *nextseg;
    Segment curseg(-1,0,-1,0);
    while(1) {
      if( (sega!=sega_end)&&(segb!=segb_end) ) {
        if( sega->L < segb->L ) {
          nextseg=sega;
          sega++;
        }
        else {
          nextseg=segb;
          segb++;
        }
      } else {
        if( sega != sega_end ) {
          nextseg=sega;
          sega++;
        } else if( segb != segb_end ) {
          nextseg=segb;
          segb++;
        }
        else {
          if( curseg.L>=0 ) {
            segout->append( curseg );
          }
          break;
        }
      }

      if( curseg.R > nextseg->L ) {
        if( curseg.R < nextseg->R ) {
          curseg.R=nextseg->R;
        }
      } else {
        if( curseg.L >= 0 ) {
          segout->append( curseg );
        }
        curseg.L=nextseg->L;
        curseg.R=nextseg->R;
        curseg.y=nextseg->y;
        curseg.id=0;
      }
    }

  }

public:
  void operator=(const Segments &a){
    segs=a.segs;
    index=a.index;
    w=a.w;
    h=a.h;
  }
  Segments(const Segments<Container> &a):segs(a.segs),index(a.index),w(a.w),h(a.h) {
  }
  Segments():segs(std::make_shared<Container<Segment>>()),index(std::make_shared<std::vector<short>>()) {}
  Segments(const int width,const int height):w(width),h(height),segs(std::make_shared<Container<Segment>>()),index(std::make_shared<std::vector<short>>(height+1)){}
  Segments(const unsigned char *pbw,const int width,const int height, const int bytesPerPixel,const int bytesPerLine,const unsigned char BIT)
  :w(width),h(height),segs(std::make_shared<Container<Segment>>()),index(std::make_shared<std::vector<short>>(height+1)){
    init(pbw,width,height,bytesPerPixel,bytesPerLine,BIT);
  }
  void init(const unsigned char *bw,const int width,const int height) {
    init(bw,width,height,1,width,0x01);
  }
  void init(const unsigned char *pbw,const int width,const int height, const int bytesPerPixel,const int bytesPerLine,const unsigned char BIT) {
    index->reserve(height+1);
    h=height;
    w=width;
    for(int y=0;y<height;y++) {
      int x=0;
      const unsigned char *bw = &pbw[bytesPerLine * y];
      (*index)[y]=segs->size();
      while(x<width) {
        while( ((*bw&BIT)==0)&&(x<width) ) {
          bw+=bytesPerPixel;
          x++;
        }
        if( ((*bw&BIT)==0)&&(x>=width) ) {
          break;
        }
        Segment seg=Segment(x,-1,y,-1);
        while((*bw&BIT)&&(x<width) ) {
          bw+=bytesPerPixel;
          x++;
        }
        seg.R=x-1;
        if( seg.L<=seg.R) {
          append(seg);
        }
      }
    }
    (*index)[h]=segs->size();
  }
  /**
  * @fn clone object
  */
  Segments<Container> clone() {
    Segments<Container> segments;
    segments.w=this->w;
    segments.h=this->h;
    segments.index = this->index.clone();
    segments.segs = this->segs.clone();

    return segments;
  }
  /**
  * @fn invert binary image
  */
  Segments<Container> invert() {
    Segments<Container> segments(this->w,this->h);

    int cury=0;
    int curx=0;
    (*segments.index)[0]=0;
    for(auto it=segs->begin();it!=segs->end();it++) {
      if( cury != it->y ) {
        if( curx < w ) {
          segments.append( Segment(curx,w-1,cury,0) );
        }
        curx=0;
        cury=it->y;

        while( cury != it->y ) {
          cury++;
          (*segments.index)[cury]=segments.segmentSize();
        }
      }
      Segment seg(curx,it->L-1,cury,0);
      curx=it->R+1;
      if( seg.L <= seg.R ) {
        segments.append( seg );
      }
    }
    (*segments.index)[h]=segments.segmentSize();
    return segments;
  }

  /**
  * @fn convert segments to binary image
  * @param (bytesPerPixel) bytes per pixel
  * @param (bytesPerLine) bytes per line
  */
  std::shared_ptr<unsigned char> convertToImage(const int bytesPerPixel,const int bytesPerLine) {
    std::shared_ptr<unsigned char> img=std::make_shared<unsigned char>(bytesPerLine*this->h);
    this->convertToImage(&*img,bytesPerPixel,bytesPerLine);
    return img;
  }

  /**
  * @fn convert segments to binary image
  * @param (img) image to write
  * @param (bytesPerPixel) bytes per pixel
  * @param (bytesPerLine) bytes per line
  */
  void convertToImage(unsigned char *img,const int bytesPerPixel,const int bytesPerLine){
    memset(img,0,h*bytesPerLine);
    for(auto it=segs->begin();it!=segs->end();it++) {
      memset( &img[ bytesPerLine * it->y + bytesPerPixel * it->L ], 0xff, (it->R-it->L+1)*bytesPerPixel );
    }
  }


  /**
  * @fn convert segments to color image based on id
  * @param (bytesPerPixel) bytes per pixel
  * @param (bytesPerLine) bytes per line
  * @param (colors) color table (Ncolors*bytesPerPixel) bytes
  * @param (Ncolors) number of colors
  */
  std::shared_ptr<unsigned char> convertToImage(const int bytesPerPixel,const int bytesPerLine,const unsigned char *colors,const int Ncolors) {
    std::shared_ptr<unsigned char> img=std::make_shared<unsigned char>(bytesPerLine*this->h);
    this->convertToImage(&*img,bytesPerPixel,bytesPerLine,colors,Ncolors);
    return img;
  }

  /**
  * @fn convert segments to color image based on id
  * @param (img) image to write
  * @param (bytesPerPixel) bytes per pixel
  * @param (bytesPerLine) bytes per line
  * @param (colors) color table (Ncolors*bytesPerPixel) bytes
  * @param (Ncolors) number of colors
  */
  void convertToImage(unsigned char *img,const int bytesPerPixel,const int bytesPerLine,const unsigned char *colors,const int Ncolors){
    memset(img,0,h*bytesPerLine);
    int i=0;
    for(auto it=segs->begin();it!=segs->end();it++,i++) {
      unsigned char *pimg=&img[bytesPerLine*it->y+bytesPerPixel*it->L];
      for(int x=it->L;x<=it->R;x++,pimg+=bytesPerPixel) {
        memcpy( pimg , &colors[((it->id)%Ncolors)*bytesPerPixel] , bytesPerPixel );
      }
    }
  }

  /**
  * @fn and operator
  * AND-opration
  */
  Segments<Container> operator&(Segments<Container> &a) {
    Segments<Container> ret( std::max(w,a.w), std::max(h,a.h) );
    for(int y=0;y<ret.h;y++) {
      (*ret.index)[y]=ret.segs->size();
      and_line( *ret.segs, &(*segs)[(*index)[y]],&(*segs)[(*index)[y+1]],
                        &(*a.segs)[(*a.index)[y]],&(*a.segs)[(*a.index)[y+1]]);
    }
    return ret;
  }

  /**
  * @fn or operator
  * OR-opration
  */
  Segments<Container> operator|(Segments<Container> &a) {
    Segments<Container> ret( std::max(w,a.w), std::max(h,a.h) );
    int h_or=std::min(h,a.h);

    for(int y=0;y<h_or;y++) {
      (*ret.index)[y]=ret.segs->size();
      or_line( ret.segs, &(*segs)[(*index)[y]],&(*segs)[(*index)[y+1]],
                        &(*a.segs)[(*a.index)[y]],&(*a.segs)[(*a.index)[y+1]]);
    }

    if( h > h_or ) {
      Segment *pseg=&(*segs)[(*index)[h_or]];
      Segment *psegend=&(*segs)[segs->size()];
      while( pseg != psegend ) {
        append( *pseg );
        pseg++;
      }
    }
    if( a.h > h_or ) {
      Segment *pseg=&(*a.segs)[(*a.index)[h_or]];
      Segment *psegend=&(*a.segs)[a.segs->size()];
      while( pseg != psegend ) {
        append( *pseg );
        pseg++;
      }
    }
    return ret;
  }

  /**
  * @fn erode operation
  */
  Segments<Container> erode() {
    Segments<Container> ret(w,h);
    (*ret.index)[0]=0;
    for(int y=0;y<h-1;y++) {
      (*ret.index)[y+1]=ret.segs->size();
      Segment *pseg0=&(*segs)[(*index)[y]];
      Segment *pseg1=&(*segs)[(*index)[y+1]];
      Segment *pseg2=&(*segs)[(*index)[y+2]];
      and_line_shrink(&*(ret.segs),pseg0,pseg1,pseg1,pseg2);
    }
    (*ret.index)[h]=ret.segs->size();
    return ret;
  }

  /**
  * @fn dilate operation
  */
  Segments<Container> dilate() {
    Segments<Container> ret(w,h);
    for(int y=0;y<h-1;y++) {
      Segment *pseg0=&(*segs)[(*index)[y]];
      Segment *pseg1=&(*segs)[(*index)[y+1]];
      Segment *pseg2=&(*segs)[(*index)[y+2]];
      or_line_expand(&*(ret.segs),pseg0,pseg1,pseg1,pseg2);
    }
    return ret;
  }

  /**
  * @fn open operation
  */
  Segments<Container> open() {

  }

  /**
  * @fn close operation
  */
  Segments<Container> close() {

  }


  /**
  * @fn reset id of all segments
  */
  void resetId(short id) {
    for(auto seg : (*segs) ) {
      seg.id = id;
    }
  }

  /**
  *@fn obtain container pointer
  */
  std::shared_ptr<Container<Segment>> container(){return segs;}
  /** @fn segment size*/
  int segmentSize(){return segs->size();}
  /** @fn image width */
  int imageWidth(){return w;}
  /** @fn image height*/
  int imageHeight(){return h;}
  /** @fn top of segment pointer of y */
  Segment *begin(int y) {return &(*segs)[(*index)[y]];}
  /** @fn end of segment pointer of y */
  Segment *end(int y) {return begin(y+1);}
  void showInfo() {
    std::cout << " Size:" << segs->size() << std::endl;
  }
};

#endif

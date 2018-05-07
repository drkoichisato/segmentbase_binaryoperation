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
* @file Labeling and blob operation
*/

#ifndef BLOB_H
#define BLOB_H

#include "segment.h"

class StatBasic {
private:
  long sumx;
  long N;
  long sumy;
public:
  StatBasic() : sumx(0),N(0),sumy(0) {}
  void init() {
    sumx=0;
    N=0;
    sumy=0;
  }
  void append( Segment seg ) {
    int n=seg.R-seg.L+1;
    N+=n;
    sumy += seg.y * n;
    sumx += (seg.R+seg.L)*n/2;
  }
};



template <class StatModel>
class Blob {
private:
  StatModel statmodel;
  unsigned short id;

public:
    Blob():id(0) {}
    Blob(unsigned short newid) :id(newid) {}
    void operator+=(Segment seg) {
      statmodel.append(seg);
      seg.id=id;
    }
};

template< template<class T, class Allocator=std::allocator<T> > class Container,class StatModel>
class Blobs {
private:
  std::shared_ptr<Container<Blob<StatModel>> > blobs;

public:
  Blobs(){};
  Blobs(Segments<Container> &segs ):blobs(std::make_shared<Container<Blob<StatModel>>>()) {
    labeling(segs);
  }
  Blobs(const Blobs<Container,StatModel> &a):blobs(a) {

  }
  void operator=(const Blobs<Container,StatModel> &a) {
    blobs = a.blobs;
  }

  void newblob(Segment *seg,std::unordered_map<int,int> *label) {
    int id=blobs->size()+1;
    Blob<StatModel> b(id);
    seg->id=id;
    b+=*seg;
    blobs->push_back(b);
    (*label)[id]=id;
  }
  int labeling(Segments<Container> &segs) {
    Segment *pseg=segs.begin(0);
    Segment *pseg_end=segs.end(0);
    Segment *cseg=segs.begin(0);
    Segment *cseg_end=segs.end(0);
    std::unordered_map<int,int> label;

    segs.resetId(0);

    for(;cseg<cseg_end;cseg++) {
      newblob(cseg,&label);
    }

    for(int y=1;y<segs.imageHeight();y++){
      cseg=segs.begin(y);
      cseg_end=segs.end(y);
      pseg=segs.begin(y-1);
      pseg_end=segs.end(y-1);



      while( (pseg!=pseg_end)&&(cseg!=cseg_end) ){
        if( cseg->isConnecting4(*pseg) ){
          cseg->connect(*pseg,&label);
        }
        if( pseg->R < cseg->R )
          pseg++;
        else
        {
          if( cseg->id == 0 ){
            newblob(cseg,&label);
          }
          cseg++;
        }
      }

      for(;cseg!=cseg_end;cseg++) {
        if( cseg->id == 0)
          newblob(cseg,&label);
      }

    }


    int id=1;
    for(int i=1;i<label.size();i++) {
      if( label[i]==i ) {
        label[i]=id;
        id++;
      } else {
        int newid=label[i];
        while( newid != label[newid] ) {
          newid=label[newid];
        }
        label[i]=newid;
      }
    }

    for(pseg=segs.begin(0);pseg!=segs.end(segs.imageHeight()-1);pseg++){
      pseg->id=label[pseg->id];
    }

    return 0;
  }
};





#endif

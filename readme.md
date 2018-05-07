# Fast Binary Image Operation Library

Segment-based binary operation library is binary operation library such as labeling, erosion, and dilation.
This library first converts binary images to horizontal segments, then apply operations over segments.
Compared to pixel by pixel operation, this library is fast and small memory usage.

## Getting Started

Source file consists of only three files, segment.h, blob.h and OpenCVWP.h.
segment.h is for segment operations
blob.h is for blob operations
OpenCVWP.h is for Wrapper class for OpenCV library.



### Installing


Just copy three files to your header folder and include them to your source code.
(If you don't use opencv, you do not have to copy OpenCVWP.h)

## Running the tests

Example file is located in example folder.


example/main.cpp is a simple example using opencv and Lenna image.


```
cd example
make
./main
```


## Authors

* **Koichi Sato** - [Koichi Sato](https://github.com/drkoichisato)

## License

This project is licensed under the LGPL 2.1+ - see the [COPYING.txt](COPYING.txt) file for details

## Acknowledgments

* Special thanks to Hyohoon Choi (Contributor)

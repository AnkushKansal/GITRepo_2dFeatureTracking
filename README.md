# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Overview of the workflow
1. Load the images into a ring buffer of size 2. Kept buffer of 2 to compare among two images. 
1. Use OpenCV to apply a variety of keypoint detectors.
    - Shi-Tomasi
    - Harris
    - FAST
    - BRISK
    - ORB
    - AKAZE
    - SIFT 
1. Use OpenCV to extract keypoint descriptors.
    - BRISK
    - BRIEF
    - ORB
    - FREAK
    - AKAZE
    - SIFT 
1. Use FLANN and kNN to improve on the brute force matching of keypoint descriptors.
1. Finally, run these algorithms in various combinations to compare performance benchmarks.

It's important to distinguish between the terms of art keypoint **detector** and keypoint **descriptor**. From Udacity's lecture notes:
> - A keypoint (sometimes also interest point or salient point) detector is an algorithm that chooses points from an image based on a local maximum of a function, such as the "cornerness" metric we saw with the Harris detector.
> - A descriptor is a vector of values, which describes the image patch around a keypoint. There are various techniques ranging from comparing raw pixel values to much more sophisticated approaches such as histograms of gradient orientations.

## Building and running the project
```
mkdir build && cd build
cmake ..
make
./2D_feature_tracking
```

## Writeup, Task MP.0

### MP.1 Data buffer optimization
The double ended queue `std::deque` offers constant time O(1) insertion and deletion of objects at each end of the queue. Checking the current size of the `deque` is similarly constant time. New items are added with `.push_back()` and, once the specified ring buffer size is reached, old items are dropped with `.pop_font()`.

We could also use std::vector, erase function if the size reaches 2 to pop the item, but vector redundantly copies its items to other memory location while inserting or erase which changes its size memory if memory not reserved manully. In case of heavy data, this approach is not optimal.

### MP.2 Keypoint detection
_Lines 81-87 in MidTermProject_Camera_Student.cpp_
```
// Uncomment to relevant to set detector selection

        string detectorType = "SHITOMASI";
        // string detectorType = "HARRIS";
        //string detectorType = "FAST";
        // string detectorType = "BRISK";
        // string detectorType = "ORB";
        // string detectorType = "AKAZE";
        // string detectorType = "SIFT";  
```
### MP.3 Keypoint removal
_Lines 134-144 in MidTermProject_Camera_Student.cpp_
```
if (bFocusOnVehicle)
        {
            vector<cv::KeyPoint> filteredKeypoints;
            for (auto kp : keypoints) {
                if (vehicleRect.contains(kp.pt)) filteredKeypoints.push_back(kp);
            }
            keypoints = filteredKeypoints;
        }
        
 Pushed the need keypoints into a separate vector and later copied to source.
```

### MP.4 Keypoint descriptors
_Lines 180-185 in MidTermProject_Camera_Student.cpp_
```
        // string descriptorType = "BRISK";
        // string descriptorType = "BRIEF";
        // string descriptorType = "ORB"; // Fails with SIFT detector
        // string descriptorType = "FREAK";
        // string descriptorType = "AKAZE";  // Fails with all non-AKAZE detectors
           string descriptorType = "SIFT";  
```

### MP.5 Descriptor matching
The function `matchDescriptors` in `matching2D_Student.cpp` contains a kind of decision tree if-else structure, based on the settings of these string parameters:
- `descriptorCategory` either: `DES_BINARY` (binary), `DES_HOG` (histogram of gradients)
- `matcherType` either: `MAT_FLANN` (cv::FlannBasedMatcher), `MAT_BF` (brute force)
- `selectorType` either: `SEL_NN` (nearest neighbors), `SEL_KNN` (k nearest neighbors)

To reduce the complexity and chance of mismatching the descriptor category, made `descriptorCategory` conditional on the `descriptorType`. In this exercise, SIFT is the only histogram of gradients (HoG) based descriptor evaluated.

_Lines 205-214 in MidTermProject_Camera_Student.cpp_
```
/* For descriptor type, select binary (BINARY) or histogram of gradients (HOG) */
/* BINARY descriptors include: BRISK, BRIEF, ORB, FREAK, and (A)KAZE. */
/* HOG descriptors include: SIFT (and SURF and GLOH, all patented). */
string descriptorCategory {};
if (0 == descriptorType.compare("SIFT")) {
    descriptorCategory = "DES_HOG";
}
else {
    descriptorCategory = "DES_BINARY";
}
```
For the performance benchmarks (TASK MP.7-9) below, `matcherType` was set to `MAT_BF` and `selectorType` was set to `SEL_KNN`, which implements match filtering based on the descriptor distance ratio.

### MP.6 Descriptor distance ratio
_Lines 75-82 in matching2D_Student.cpp_

for each keypoint form matches, do below
            if ((best_match.distance / second_best_macth.distance) < 0.8)
            {
                selecting the best match
            }
     

This distance ratio filter compares the distance (SSD) between two candidate matched keypoint descriptors. A threshold of `0.8` is applied and the stronger candidate (minimum distance) is selected as the correct match. This method eliminates many false-positive keypoint matches.

### MP.7 Performance evaluation 1
The number of keypoints within the bounding box of the preceding vehicle were counted for each detector type.

See the results in: **results/TASK_MP.7.xlsx**

Harris had the fewest relevant keypoints, while the top three keypoint detectors were:
1. FAST (average of 409.4 keypoints per image)
1. BRISK (average of 276.2 keypoints per image)
1. AKAZE (average of 167 keypoints per image)

### MP.8 Performance evaluation 2
The number of matched keypoints were then counted for each valid detector type and descriptor type combination, 35 in total. Note that AKAZE descriptors works only with AKAZE detectors, and SIFT detectors coud not work with ORB descriptors.

See the results in: **results/TASK_MP.8.xlsx**

The FAST detectors with BRIEF, SIFT, and ORB descriptors consistently produced the largest number of matched keypoints (~300 per image).

### MP.9 Performance evaluation 3

With FAST detectors (broadly the fastest), the three fastest descriptors were( in increasing order of max total time taken):
1. FREAK
2. BRISK
3. BRIEF

All of these combinations consistently ran in less than `7.5 ms` average total time. See the results in: **results/TASK_MP.8.xlsx**

However, processing time must be considered with the number of keypoints successfully matched in that time. For overall performance relevant to this project, the top three combinations were:

- FAST detectors and BRIEF descriptors
- FAST detectors and ORB descriptors
- FAST detectors and SIFT descriptors

There are two disadvantages to SIFT: It is heavily patented and as it is HoG based algo, it has the worst-case execution time. For the combination, some SIFT runtimes slipped to `8 ms` total time. The runtimes for FAST with ORB and BRIEF descriptors was more tightly bound within `6.5 ms`.

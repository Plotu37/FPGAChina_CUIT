#ifndef STITCH_H
#define STITCH_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "imgproc/imgproc.hpp"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/xfeatures2d/nonfree.hpp"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/stitching.hpp>

#include<vector>
#include "opencv2/features2d.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
using namespace cv::xfeatures2d;
using namespace std;
using namespace cv;

Point2f getTransformPoint(const Point2f originalPoint, const Mat &transformMaxtri);

Mat stitch_surf(Mat image01,Mat image02);
void stitch_orb(std::vector<cv::Mat> imgs, cv::Mat& resultMat);
Mat stitch_sift(Mat image01,Mat image02);//sift

#endif // STITCHING_H

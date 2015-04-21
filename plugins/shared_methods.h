#ifndef SHARED_METHODS_H_
#define SHARED_METHODS_H_

#include <opencv/cv.h>

// ED includes
#include "ed/measurement.h"
#include <ed/entity.h>

namespace ed
{
namespace perception
{

void prepareMeasurement(const ed::EntityConstPtr& e, cv::Mat& cropped_image, cv::Mat& depth_image, cv::Mat& mask, cv::Rect& bouding_box);

float getAverageDepth(cv::Mat& depth_img);

void optimizeContourHull(const cv::Mat& mask_orig, cv::Mat& mask_optimized);

void optimizeContourBlur(const cv::Mat& mask_orig, cv::Mat& mask_optimized);

cv::Mat maskImage(const cv::Mat& img, const ed::ImageMask& mask, cv::Rect& roi);

void saveDebugImage(const std::string& name, const cv::Mat& img);

}

}

#endif

#ifndef SHARED_METHODS_H_
#define SHARED_METHODS_H_

#include <opencv2/highgui/highgui.hpp>

// ED includes
#include "ed/measurement.h"
#include <ed/entity.h>

namespace ed
{
namespace perception
{

    void prepareMeasurement(const ed::EntityConstPtr& e, cv::Mat& cropped_image, cv::Mat& depth_image, cv::Mat& mask, cv::Rect& bouding_box);

    float getMedianDepth(cv::Mat& depth_img);

    // create a new mask based on the convex hull of the original mask
    void optimizeContourHull(const cv::Mat& mask_orig, cv::Mat& mask_optimized);

    // create a new mask based on a blured version of the original mask, smoother and expanded
    void optimizeContourBlur(const cv::Mat& mask_orig, cv::Mat& mask_optimized);

    // clip an integer value
    int clipInt(int val, int min, int max);

    cv::Mat maskImage(const cv::Mat& img, const ed::ImageMask& mask, cv::Rect& roi);

    void saveDebugImage(const std::string& name, const cv::Mat& img);

    void cleanDebugFolder(std::string folder);

    cv::Mat resizeSameRatio(const cv::Mat& img, int target_width);

}
}

#endif

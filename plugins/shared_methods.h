
#ifndef SHARED_METHODS_H_
#define SHARED_METHODS_H_

#include <opencv/cv.h>

// ED includes
#include "ed/measurement.h"
#include <ed/entity.h>

class SharedMethods{

public:

    SharedMethods();

    ~SharedMethods();

    void prepareMeasurement(const ed::EntityConstPtr& e, cv::Mat& cropped_image, cv::Mat& depth_image, cv::Mat& mask, cv::Rect& bouding_box) const;

    float getAverageDepth(cv::Mat& depth_img) const;

    void optimizeContourHull(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const;

    void optimizeContourBlur(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const;

private:

protected:

};

#endif

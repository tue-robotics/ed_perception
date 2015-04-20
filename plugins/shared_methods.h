
#ifndef SHARED_METHODS_H_
#define SHARED_METHODS_H_

#include <opencv/cv.h>

// ED includes
#include "ed/measurement.h"
#include <ed/entity.h>

class SharedMethods{

public:

    /*
    * ###########################################
    *  				    PUBLIC
    * ###########################################
    */

    SharedMethods();

    ~SharedMethods();

    void prepareMeasurement(const ed::EntityConstPtr& e, cv::Mat& cropped_image, cv::Mat& depth_image, cv::Mat& mask, cv::Rect& bouding_box) const;

    float getAverageDepth(cv::Mat& depth_img) const;

    // create a new mask based on the convex hull of the original mask
    void optimizeContourHull(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const;

    // create a new mask based on a blured version of the original mask, smoother and expanded
    void optimizeContourBlur(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const;

    // clip an integer value
    int clipInt(int val, int min, int max) const;


private:

    /*
     * ###########################################
     *  				PRIVATE
     * ###########################################
     */


protected:


};

#endif

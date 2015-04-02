/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: March 2015
*/

#include "shared_methods.h"

// ED includes
#include <ed/error_context.h>
#include <rgbd/Image.h>
#include <rgbd/View.h>

// ---------------------------------------------------------------------------------------------------

SharedMethods::SharedMethods(){
}


SharedMethods::~SharedMethods(){
}

// ---------------------------------------------------------------------------------------------------

void SharedMethods::prepareMeasurement(const ed::EntityConstPtr& e, cv::Mat& cropped_image, cv::Mat& depth_image, cv::Rect& bouding_box) const{

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->lastMeasurement();
    if (!msr)
        return;

    int min_x, max_x, min_y, max_y;

    // create a view
    rgbd::View view(*msr->image(), msr->image()->getRGBImage().cols);

    // get depth image
    depth_image = msr->image()->getDepthImage();

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // crop it to match the view
    cropped_image = cv::Mat(color_image(cv::Rect(0, 0, view.getWidth(), view.getHeight())));

    std::cout << "image: " << cropped_image.cols << "x" << cropped_image.rows << std::endl;

    // initialize bounding box points
    max_x = 0;
    max_y = 0;
    min_x = view.getWidth();
    min_y = view.getHeight();

    cv::Mat mask = cv::Mat::zeros(view.getHeight(), view.getWidth(), CV_8UC1);
    // Iterate over all points in the mask
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(view.getWidth()); it != msr->imageMask().end(); ++it)
    {
        // mask's (x, y) coordinate in the depth image
        const cv::Point2i& p_2d = *it;

        // paint a mask
        mask.at<unsigned char>(*it) = 255;

        // update the boundary coordinates
        if (min_x > p_2d.x) min_x = p_2d.x;
        if (max_x < p_2d.x) max_x = p_2d.x;
        if (min_y > p_2d.y) min_y = p_2d.y;
        if (max_y < p_2d.y) max_y = p_2d.y;
    }

    bouding_box = cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y);
}


// ----------------------------------------------------------------------------------------------------


void SharedMethods::OptimizeContourHull(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const{

    std::vector<std::vector<cv::Point> > hull;
    std::vector<std::vector<cv::Point> > contours;

    mask_optimized = cv::Mat::zeros(mask_orig.size(), CV_8UC1);

    cv::findContours(mask_orig, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (uint i = 0; i < contours.size(); i++){
        hull.push_back(std::vector<cv::Point>());
        cv::convexHull(cv::Mat(contours[i]), hull.back(), false);

        cv::drawContours(mask_optimized, hull, -1, cv::Scalar(255), CV_FILLED);
    }
}


// ----------------------------------------------------------------------------------------------------


void SharedMethods::OptimizeContourBlur(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const{

    mask_orig.copyTo(mask_optimized);

    // blur the contour, also expands it a bit
    for (uint i = 6; i < 18; i = i + 2){
        cv::blur(mask_optimized, mask_optimized, cv::Size( i, i ), cv::Point(-1,-1) );
    }

    cv::threshold(mask_optimized, mask_optimized, 50, 255, CV_THRESH_BINARY);
}


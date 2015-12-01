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

#include <boost/filesystem.hpp>

namespace ed
{
namespace perception
{

// ---------------------------------------------------------------------------------------------------

void prepareMeasurement(const ed::EntityConstPtr& e, cv::Mat& view_color_img, cv::Mat& view_depth_img, cv::Mat& mask, cv::Rect& bouding_box) {

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->lastMeasurement();
    if (!msr)
        return;

    int min_x, max_x, min_y, max_y;

    // create a view
    rgbd::View view(*msr->image(), msr->image()->getRGBImage().cols);

    // get depth image
    view_depth_img = msr->image()->getDepthImage();

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // crop it to match the view
    view_color_img = cv::Mat(color_image(cv::Rect(0, 0, view.getWidth(), view.getHeight())));

//    std::cout << "image: " << cropped_image.cols << "x" << cropped_image.rows << std::endl;

    // initialize bounding box points
    max_x = 0;
    max_y = 0;
    min_x = view.getWidth();
    min_y = view.getHeight();

    // initialize mask, all 0s
    mask = cv::Mat::zeros(view.getHeight(), view.getWidth(), CV_8UC1);

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

float getMedianDepth(cv::Mat& depth_img)
{
    float median = 0;
    std::vector<float> depths;

    // fill vector with depth values
    for (uint x = 0 ; x < depth_img.cols ; x++){
        for (uint y = 0 ; y < depth_img.rows ; y++){
            if (depth_img.at<float>(y,x) > 0.0){
                depths.push_back(depth_img.at<float>(y,x));
            }
        }
    }

    if (depths.empty()){
        std::cout << "GetAverageDepth: no depth values pushed. Empty image matrix?" << std::endl;
        return 0.0;
    }

    // sort and pick the center value
    std::sort(depths.begin(),depths.end());

    if (depths.size() % 2 == 0){
        median = (depths[depths.size()/2-1] + depths[depths.size()/2]) / 2.0;
    }
    else{
        median = depths[depths.size() / 2];
    }

    return median;
}

// ----------------------------------------------------------------------------------------------------

void optimizeContourHull(const cv::Mat& mask_orig, cv::Mat& mask_optimized) {

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

void optimizeContourBlur(const cv::Mat& mask_orig, cv::Mat& mask_optimized) {

    mask_orig.copyTo(mask_optimized);

    // blur the contour, also expands it a bit
    for (uint i = 6; i < 18; i = i + 2){
        cv::blur(mask_optimized, mask_optimized, cv::Size( i, i ), cv::Point(-1,-1) );
    }

    cv::threshold(mask_optimized, mask_optimized, 50, 255, CV_THRESH_BINARY);
}


// ----------------------------------------------------------------------------------------------------


int clipInt(int val, int min, int max)
{
    return val <= min ? min : val >= max ? max : val;
}


// ----------------------------------------------------------------------------------------------------


cv::Mat maskImage(const cv::Mat& img, const ed::ImageMask& mask, cv::Rect& roi)
{
    // initialize bounding box points
    uint max_x = 0;
    uint max_y = 0;
    uint min_x = img.cols;
    uint min_y = img.rows;

    // Created masked image
    cv::Mat masked_img = cv::Mat::zeros(img.rows, img.cols, img.type());

    for(ed::ImageMask::const_iterator it = mask.begin(img.cols); it != mask.end(); ++it)
    {
        const cv::Point2i& p_2d = *it;

        masked_img.at<cv::Vec3b>(p_2d) = img.at<cv::Vec3b>(p_2d);

        // update the boundary coordinates
        if (min_x > p_2d.x) min_x = p_2d.x;
        if (max_x < p_2d.x) max_x = p_2d.x;
        if (min_y > p_2d.y) min_y = p_2d.y;
        if (max_y < p_2d.y) max_y = p_2d.y;
    }

    roi = cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y);

    return masked_img;
}


// ----------------------------------------------------------------------------------------------------


void saveDebugImage(const std::string& name, const cv::Mat& img)
{
    ed::UUID id = ed::Entity::generateID();
    std::string filename = "/tmp/" + name + "-" + id.str() + ".jpg";

    if (img.type() == CV_32FC1)
    {
        // depth image
        float d_min = 1e9;
        float d_max = 0;

        cv::Mat rgb_image(img.rows, img.cols, CV_8UC3, cv::Scalar(100, 0, 0));
        for(unsigned int i = 0; i < img.rows * img.cols; ++i)
        {
            float d = img.at<float>(i);
            if (d == d && d > 0)
            {
                d_min = std::min(d, d_min);
                d_max = std::max(d, d_max);
            }
        }

        for(unsigned int i = 0; i < img.rows * img.cols; ++i)
        {
            float d = img.at<float>(i);
            if (d == d && d > 0)
            {
                int c = 255 * (1.0 - (d - d_min) / (d_max - d_min));
                rgb_image.at<cv::Vec3b>(i) = cv::Vec3b(c, c, c);
            }
        }

        cv::imwrite(filename, rgb_image);
    }
    else
    {
        cv::imwrite(filename, img);
    }
}


// ----------------------------------------------------------------------------------------------------


void cleanDebugFolder(std::string folder){
    // clean the debug folder if debugging is active
    try {
        boost::filesystem::path dir(folder);
        boost::filesystem::remove_all(dir);
        boost::filesystem::create_directories(dir);
    } catch(const boost::filesystem::filesystem_error& e){
       if(e.code() == boost::system::errc::permission_denied)
           std::cout << "[" << "cleanDebugFolder" << "] " << "boost::filesystem permission denied" << std::endl;
       else
           std::cout << "[" << "cleanDebugFolder" << "] " << "boost::filesystem failed with error: " << e.code().message() << std::endl;
    }
}


// ----------------------------------------------------------------------------------------------------

cv::Mat resizeSameRatio(const cv::Mat& img, int target_width = 500){
    int width = img.cols;
    int height = img.rows;

    cv::Rect roi;
    cv::Mat square = cv::Mat(target_width, target_width, img.type() );

    square.setTo(cv::Scalar(255,255,255));

    int max_dim = ( width >= height ) ? width : height;
    float scale = ( ( float ) target_width ) / max_dim;

    if ( width >= height ){
        roi.width = target_width;
        roi.x = 0;
        roi.height = height * scale;
        roi.y = ( target_width - roi.height ) / 2;
    } else {
        roi.y = 0;
        roi.height = target_width;
        roi.width = width * scale;
        roi.x = ( target_width - roi.width ) / 2;
    }

    cv::resize( img, square( roi ), roi.size() );

    return square;
}

// ----------------------------------------------------------------------------------------------------
}
}

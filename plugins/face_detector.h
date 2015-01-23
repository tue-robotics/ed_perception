/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#ifndef ED_PERCEPTION_FACE_DETECTOR_H_
#define ED_PERCEPTION_FACE_DETECTOR_H_

#include <ed/perception_modules/perception_module.h>

// OpenCV includes
#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"

class FaceDetector : public ed::PerceptionModule
{

/*
 * ###########################################
 *  				PRIVATE
 * ###########################################
 */
private:

    // module configuration
    bool init_success_;
    bool debug_mode_;            /*!< Enable debug mode */
    std::string	module_name_;    /*!< Name of the module, for output */
    std::string debug_folder_;   /*!< Path of the debug folder */
    std::string cascade_files_path_;   /*!< Path of the cascade training folder */

    // Cascade classifier configuration
    double classif_front_scale_factor_;  // Parameter specifying how much the image size is reduced at each image scale
    int classif_front_min_neighbours_;    // Parameter specifying how many neighbors each candidate rectangle should have to retain it.
    cv::Size classif_front_min_size_;    // Minimum possible object size. Objects smaller than that are ignored.

    double classif_profile_scale_factor_;  // Parameter specifying how much the image size is reduced at each image scale
    int classif_profile_min_neighbours_;    // Parameter specifying how many neighbors each candidate rectangle should have to retain it.
    cv::Size classif_profile_min_size_;   // Minimum possible object size. Objects smaller than that are ignored.

    // Haar cascade classifiers
    mutable cv::CascadeClassifier classifier_front;
    mutable cv::CascadeClassifier classifier_profile;

    // detect frontal and profile faces on an image, true if a face was detected
    bool DetectFaces(const cv::Mat &cropped_img,
                     std::vector<cv::Rect> &faces_front,
                     std::vector<cv::Rect> &faces_profile) const;

    // clip an integer value
    int ClipInt(int val, int min, int max) const;

    // create a new mask based on the convex hull of the original mask
    void OptimizeContourHull(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const;

    // create a new mask based on a blured version of the original mask, smoother and expanded
    void OptimizeContourBlur(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const;

/*
* ###########################################
*  				    PUBLIC
* ###########################################
*/
public:

    FaceDetector();

    virtual ~FaceDetector();

    void loadConfig(const std::string& config_path);

    void process(ed::EntityConstPtr e, tue::Configuration& result) const;

};

#endif

/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#ifndef ED_PERCEPTION_FACE_DETECTOR_H_
#define ED_PERCEPTION_FACE_DETECTOR_H_

#include <ed/perception/module.h>

// OpenCV includes
#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

class FaceDetector : public ed::perception::Module
{

/*
 * ###########################################
 *  				PRIVATE
 * ###########################################
 */
private:

    // module configuration
    bool debug_mode_;            /*!< Enable debug mode */
    std::string debug_folder_;   /*!< Path of the debug folder */

    // Cascade classifier configuration
    double classifier_front_scale_factor_;  // Parameter specifying how much the image size is reduced at each image scale
    int classifier_front_min_neighbours_;    // Parameter specifying how many neighbors each candidate rectangle should have to retain it.
    cv::Size classif_front_min_size_;    // Minimum possible object size. Objects smaller than that are ignored.

    double classifier_profile_scale_factor_;  // Parameter specifying how much the image size is reduced at each image scale
    int classifier_profile_min_neighbours_;    // Parameter specifying how many neighbors each candidate rectangle should have to retain it.
    cv::Size classif_profile_min_size_;   // Minimum possible object size. Objects smaller than that are ignored.

    // Haar cascade classifiers
    mutable cv::CascadeClassifier classifier_front_;
    mutable cv::CascadeClassifier classifier_profile_;

    //------------------------------------

    // detect frontal and profile faces on an image, true if a face was detected
    bool DetectFaces(const cv::Mat &cropped_img,
                     std::vector<cv::Rect> &faces_front,
                     std::vector<cv::Rect> &faces_profile) const;



    void writeFaceDetectionResult(const ed::Measurement& msr, const cv::Rect& rgb_roi, const std::vector<cv::Rect>& rgb_face_rois,
                                  int& face_counter, tue::Configuration& result) const;

/*
* ###########################################
*  				    PUBLIC
* ###########################################
*/
public:

    FaceDetector();

    virtual ~FaceDetector();

    void configureClassification(tue::Configuration config);

    void classify(const ed::Entity& e, const std::string& property, const ed::perception::CategoricalDistribution& prior,
                  ed::perception::ClassificationOutput& output) const;

    void addTrainingInstance(const ed::Entity& e, const std::string& property, const std::string& value);

    void train() {}

    void loadRecognitionData(const std::string& path) {}

    void saveRecognitionData(const std::string& path) const {}

private:

    std::set<std::string> face_data_;

    void recognizeFace(const cv::Mat& image, const cv::Rect& roi, ed::perception::ClassificationOutput& output) const;

};

#endif

#ifndef ED_PERCEPTION_FACE_RECOGNITION_H_
#define ED_PERCEPTION_FACE_RECOGNITION_H_

#include <ed/perception_modules/perception_module.h>

// OpenCV includes
//#include <opencv/cv.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>


class FaceRecognition : public ed::PerceptionModule
{

enum{
    EIGEN = 0,
    FISHER = 1,
    LBPH = 2
};

/*
 * ###########################################
 *  				PRIVATE
 * ###########################################
 */
private:

//    std::string num_test;
//    cv::Mat test_img;
//    std::string test_name;

    // module configuration
    bool init_success_;
    bool debug_mode_;
    std::string module_name_;

    bool using_Eigen_;
    bool using_Fisher_;
    bool using_LBP_;

    // Images and corresponding labels.
    std::vector<cv::Mat> images_;
    std::vector<int> labels_;
    std::map<int, std::string> labelsInfo_;
    std::map<int, std::string> id_name_map_;
    std::map<std::string, int> name_id_map_;

    // Face recognizers
    std::vector<cv::Ptr<cv::FaceRecognizer> > models;

    void read_csv(const std::string& filename, std::vector<cv::Mat>& images_, std::vector<int>& labels_, std::map<int, std::string>& labelsInfo_, char separator = ';');

    bool isFaceFound(tue::Configuration config) const;

    bool getFaceInfo(tue::Configuration config, cv::Rect& faceRect) const;

    bool AlignFace(cv::Mat origImg, cv::Rect faceLoc, int targetSize, float horizOffset, float vertOffset, cv::Mat& faceImg) const;

    int ClipInt(int val, int min, int max) const;

    int Distance(cv::Point p1, cv::Point p2) const;

    void rotateFace(cv::Mat faceImg, cv::Mat& rotatedImg, cv::Point leftEye, cv::Point rightEye, cv::Point center) const;

/*
* ###########################################
*  				    PUBLIC
* ###########################################
*/
public:

    FaceRecognition();

    virtual ~FaceRecognition();

    void loadConfig(const std::string& config_path);

    void process(ed::EntityConstPtr e, tue::Configuration& config) const;

};

#endif

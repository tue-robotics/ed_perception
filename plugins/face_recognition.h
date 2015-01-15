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

    // -------------------- VARIABLES --------------------

    // module configuration
    bool init_success_;         // signal if initialization is complete
    bool debug_mode_;           // signal if debug mode is active, to enable output communication
    std::string module_name_;   // module name that shows up in the output

    bool using_Eigen_;          // is Eingen Faces recognition enabled
    bool using_Fisher_;         // is Fisher Faces recognition enabled
    bool using_LBPH_;            // is LBPH recognition enabled

    // face alignement parameters
    int face_target_size_;            // size of the image after alignement
    float face_vert_offset_;      // vertical offeset from the left eye to the top margin of the image
    float face_horiz_offset_;    // horizontal offeset from the left eye to the left margin of the image

    float eigen_treshold_;
    float fisher_treshold_;
    float lbph_treshold_;

    // Face recogniton training
    std::vector<cv::Mat> images_;
    std::vector<int> labels_;
    std::map<int, std::string> labelsInfo_;

    // Face recognizers
    std::vector<cv::Ptr<cv::FaceRecognizer> > models;



    // -------------------- FUNCTIONS --------------------

    // Read the CSV file with the path to the image, the class number and label
    void read_csv(const std::string& filename,
                  const std::string& images_path,
                  std::vector<cv::Mat>& images,
                  std::vector<int>& labels,
                  std::map<int, std::string>& labelsInfo,
                  char separator = ';');

    // reads the config of the entity to determine if a face was found
    bool isFaceFound(tue::Configuration config) const;

    // reads the config to get the information about a face, such as face location and eye location
    bool getFaceInfo(tue::Configuration config, cv::Rect& faceRect) const;

    // process face for recognition, alignement, grayscale, histogram equalization and cutting
    bool alignFace(cv::Mat origImg, cv::Rect faceLoc, int targetSize, float horizOffset, float vertOffset, cv::Mat& faceImg) const;

    // clip integer value
    int clipInt(int val, int min, int max) const;

    // calculate euclidean distance between two points
    int euclidDistance(cv::Point p1, cv::Point p2) const;

    // rotate a face according to the position of the eyes
    void rotateFace(cv::Mat faceImg, cv::Mat& rotatedImg, cv::Point leftEye, cv::Point rightEye, cv::Point center) const;

    // match the results from the recogniton into a single result
    void matchResults(std::string eigenLabel, std::string fisherLabel, std::string lbphLabel,
                      float eigenConf, float fisherConf, float lbphConf,
                      std::string& label_match, double& confidence_match) const;

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

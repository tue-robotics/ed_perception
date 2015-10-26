/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#ifndef ED_PERCEPTION_FACE_RECOGNITION_H_
#define ED_PERCEPTION_FACE_RECOGNITION_H_

#include <ed/perception/module.h>

#include <ed_perception/LearnPerson.h>
#include <ed_perception/LearnPersonRequest.h>
#include <ed_perception/LearnPersonResponse.h>

#include <ed_perception/FaceLearningAction.h>

// OpenCV includes
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <ros/callback_queue.h>
#include <ros/service_server.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "color_matcher/color_matcher.h"

class FaceRecognition : public ed::perception::Module
{

enum Recognizers{
    EIGEN = 0,
    FISHER = 1,
    LBPH = 2,
    HIST = 3
};

struct FaceInfo
{
    int index;
    cv::Rect features;
    cv::Mat face_img;

    std::vector<int> predicted_label;
    std::vector<std::string> predicted_name;
    std::vector<double> confidence;

    std::string face_match_result;
    double face_confidence_match;

    void initalize(){
        // initialize values
        predicted_label.resize(4);
        confidence.resize(4);
        predicted_name.resize(4);

        predicted_label[EIGEN] = -1;
        predicted_label[FISHER] = -1;
        predicted_label[LBPH] = -1;
        predicted_label[HIST] = -1;

        confidence[EIGEN] = 0.0;
        confidence[FISHER] = 0.0;
        confidence[LBPH] = 0.0;
        confidence[HIST] = 0.0;

        predicted_name[EIGEN] = "";
        predicted_name[FISHER] = "";
        predicted_name[LBPH] = "";
        predicted_name[HIST] = "";

        face_match_result = "";
        face_confidence_match = 0;
    }

    FaceInfo(){this->initalize();}

    FaceInfo(int index, cv::Rect face_rect){
        features = face_rect;
        this->index = index;
        this->initalize();
    }
};

/*
 * ###########################################
 *  				PRIVATE
 * ###########################################
 */
private:

    // -------------------- VARIABLES --------------------

    // module configuration
    bool init_success_;             // signal if initialization is complete
    bool debug_mode_;               // signal if debug mode is active, to enable output communication
    std::string module_name_;       // module name that shows up in the output
    std::string module_path_;       // module name that shows up in the output
    mutable bool learning_mode_;    // true if face is being learned, false for normal recognition mode
    std::string saved_faces_dir_;    // directory where to save the faces learned, for later re-learning
    bool save_learned_faces_;        // wheter to save the faces learned or not

    // face alignement parameters
    int face_target_size_;      // size of the image after alignement
    float face_vert_offset_;    // vertical offeset from the left eye to the top margin of the image
    float face_horiz_offset_;   // horizontal offeset from the left eye to the left margin of the image

    float eigen_treshold_;      // treshold for a trustworthy classification with Eigen Faces
    float fisher_treshold_;     // treshold for a trustworthy classification with Fisher Faces
    float lbph_treshold_;       // treshold for a trustworthy classification with LBPH Faces
    double recognition_treshold_;

    // Face recogniton training
    mutable std::vector<cv::Mat> images_;               // images used for training
    mutable std::vector<int> labels_;                   // label corresponding to each image
    mutable std::map<int, std::string> labels_info_;    // connects a label number with a person name
    mutable std::map<int, std::vector<cv::Mat> > learned_histograms_;    // connects label number name with a set of colors
    mutable int last_label_;                            // last label number used

    // Face recognizers
    mutable std::vector<cv::Ptr<cv::FaceRecognizer> > models_;      // vector of models for the FaceRecognizers

    bool using_Eigen_;          // is Eingen Faces recognition enabled
    bool using_Fisher_;         // is Fisher Faces recognition enabled
    bool using_LBPH_;           // is LBPH recognition enabled
    bool using_histogram_;      // is histogram comparison enabled

    bool enable_learning_service_;

    mutable bool trained_Eigen_;          // is Eingen Faces recognition enabled
    mutable bool trained_Fisher_;         // is Fisher Faces recognition enabled
    mutable bool trained_LBPH_;           // is LBPH recognition enabled

    // learning service
    mutable ros::CallbackQueue cb_queue_;   // service queue
    ros::ServiceServer srv_learn_face;      // service to learn new face
    std::string learning_name_;             // name to be used in the learning process
    mutable uint n_faces_current_;          // number of faces learned for the last request
    int max_faces_learn_;                  // total number of faces to be learned for each model


    actionlib::SimpleActionServer<ed_perception::FaceLearningAction> *as_;


    // -------------------- FUNCTIONS --------------------

    // Read the CSV file with the path to the image, the class number and label
    void readCSV(const std::string& filename,
                  std::vector<cv::Mat>& images,
                  std::vector<int>& labels,
                  std::map<int, std::string>& labelsInfo,
                  char separator = ';');

    // reads the config of the entity to determine if a face was found
    bool isFaceFound(tue::Configuration config) const;

    // reads the config to get the information about a face, such as face location and eye location
    bool getFacesInfo(tue::Configuration config, std::vector<FaceInfo>& faces) const;

    // process face for recognition, alignement, grayscale, histogram equalization and cutting
    bool alignFace(cv::Mat origImg, cv::Rect faceLoc, int targetSize, float horizOffset, float vertOffset, cv::Mat& faceImg) const;

    // clip integer value
    int clipInt(int val, int min, int max) const;

    // calculate euclidean distance between two points
    int euclidDistance(cv::Point p1, cv::Point p2) const;

    // rotate a face according to the position of the eyes
    void rotateFace(cv::Mat faceImg, cv::Mat& rotatedImg, cv::Point leftEye, cv::Point rightEye, cv::Point center) const;

    // match the results from the recogniton into a single result
    void matchClassifications(std::vector<std::string> classifications,
                              std::vector<double> confidence,
                              std::string& label_match,
                              double& confidence_match) const;

    // show visualization window with information
    void showDebugWindow(cv::Mat face_aligned,
                         std::vector<std::string> predicted_name,
                         std::vector<double> confidence,
                         std::string face_match,
                         double face_confidence) const;

    // function called when service is requested
    bool srvStartLearning(const ed_perception::LearnPerson::Request& ros_req, ed_perception::LearnPerson::Response& ros_res);

    // learns a face
    bool learnFace(std::string person_name,
                   int person_label,
                   cv::Mat& face,
                   cv::Mat& histogram,
                   int n_face,
                   std::vector<cv::Mat>& face_images,
                   std::vector<int>& face_labels,
                   std::map<int, std::string>& faces_info) const;

    // trains the opencv FaceRecognizers
    void trainRecognizers(std::vector<cv::Mat>& images, std::vector<int>& labels, std::vector<cv::Ptr<cv::FaceRecognizer> >& models) const;

    // construct an histogram from the entities colors present in the config
    void getEntityHistogram(tue::Configuration config, cv::Mat& entity_histogram) const;

    // get a matching confidence between the current entity histogram and previously learned ones
    void matchHistograms(cv::Mat& entity_histogram,
                         std::map<int, std::vector<cv::Mat> >& learned_histograms,
                         int& color_match_label,
                         double& color_match_confidence) const;

    // preempt callback from the learning action service
    void learning_as_preempt();

    // goal callback from the learning action service
    void learning_as_goal();


/*
* ###########################################
*  				    PUBLIC
* ###########################################
*/
public:

    FaceRecognition();

    virtual ~FaceRecognition();

    void loadConfig(const std::string& config_path);

    void process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const;

    void configure(tue::Configuration config);


    // New interface

    void classify(const ed::Entity& e, const std::string& property, const ed::perception::CategoricalDistribution& prior,
                  ed::perception::ClassificationOutput& output) const {}

    void addTrainingInstance(const ed::Entity& e, const std::string& property, const std::string& value) {}

    void train() {}

    void loadRecognitionData(const std::string& path) {}

    void saveRecognitionData(const std::string& path) const {}

};

#endif

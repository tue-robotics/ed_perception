/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#include "face_recognition.h"

#include "ed/measurement.h"
#include <ed/entity.h>
#include <ed/error_context.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <numeric>

#include <actionlib/server/action_server.h>


// ----------------------------------------------------------------------------------------------------

FaceRecognition::FaceRecognition() :
    ed::perception::Module("face_recognition"),
    as_(0),
    init_success_(false)
{

}


// ----------------------------------------------------------------------------------------------------

FaceRecognition::~FaceRecognition(){

    if (debug_mode_){
        cv::destroyWindow("Face Recognition debug");
    }
}


// ----------------------------------------------------------------------------------------------------


void FaceRecognition::configure(tue::Configuration config) {

    if (!config.value("debug_mode", debug_mode_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'debug_mode' not found. Using default: " << debug_mode_ << std::endl;

    if (!config.value("save_learned_faces", save_learned_faces_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'save_learned_faces' not found. Using default: " << save_learned_faces_ << std::endl;

    if (!config.value("saved_faces_dir", saved_faces_dir_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'saved_faces_dir' not found. Using default: " << saved_faces_dir_ << std::endl;

    if (!config.value("use_Eigen", using_Eigen_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'use_Eigen' not found. Using default: " << using_Eigen_ << std::endl;

    if (!config.value("use_Fisher", using_Fisher_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'use_Fisher' not found. Using default: " << using_Fisher_ << std::endl;

    if (!config.value("use_LBPH", using_LBPH_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'use_LBPH' not found. Using default: " << using_LBPH_ << std::endl;

    if (!config.value("use_histogram", using_histogram_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'use_histogram' not found. Using default: " << using_histogram_ << std::endl;

    if (!config.value("enable_learning_service", enable_learning_service_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'enable_learning_service' not found. Using default: " << enable_learning_service_ << std::endl;

    if (!config.value("eigen_treshold", eigen_treshold_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'eigen_treshold' not found. Using default: " << eigen_treshold_ << std::endl;

    if (!config.value("fisher_treshold", fisher_treshold_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'fisher_treshold' not found. Using default: " << fisher_treshold_ << std::endl;

    if (!config.value("lbph_treshold", lbph_treshold_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'lbph_treshold' not found. Using default: " << lbph_treshold_ << std::endl;

    if (!config.value("recognition_treshold", recognition_treshold_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'recognition_treshold' not found. Using default: " << recognition_treshold_ << std::endl;

    if (!config.value("face_target_size", face_target_size_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'face_target_size' not found. Using default: " << face_target_size_ << std::endl;

    if (!config.value("left_eye_vert_offset", face_vert_offset_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'left_eye_vert_offset' not found. Using default: " << face_vert_offset_ << std::endl;

    if (!config.value("left_eye_horiz_offset", face_horiz_offset_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'left_eye_horiz_offset' not found. Using default: " << face_horiz_offset_ << std::endl;

    if (!config.value("max_faces_learn", max_faces_learn_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'max_faces_learn' not found. Using default: " << max_faces_learn_ << std::endl;

    saved_faces_dir_ = (std::string)getenv("HOME") + saved_faces_dir_;

    // create debug window
    if (debug_mode_){
        cv::namedWindow("Face Recognition debug", cv::WINDOW_AUTOSIZE);
    }

    // Get the path to the CSV file and images
    std::string csv_file_path = saved_faces_dir_ + "face_models.cvs";

    // Load faces from CSV file
    if (!csv_file_path.empty()){
        readCSV(csv_file_path, images_, labels_, labels_info_);
        trainRecognizers(images_, labels_, models_);
    }else {
        std::cout << "[" << module_name_ << "] " << "No CSV faces file was loaded! Recognizers not trained" << std::endl;
    }

    // create faces folder
    if (save_learned_faces_){
        boost::filesystem::path dir(saved_faces_dir_);
        boost::filesystem::create_directories(dir);
        std::cout << "[" << module_name_ << "] " << "Faces learned will be saved in: " << saved_faces_dir_ << std::endl;
    }

    if (enable_learning_service_){
        // initialize connection with ROS
        if (!ros::isInitialized()){
            ros::M_string remapping_args;
            ros::init(remapping_args, "ed");
        }

        ros::NodeHandle nh("~");
        nh.setCallbackQueue(&cb_queue_);

        // publish service
        ros::AdvertiseServiceOptions opt_srv_get_entity_info =
                ros::AdvertiseServiceOptions::create<ed_perception::LearnPerson>(
                    "face_recognition/learn", boost::bind(&FaceRecognition::srvStartLearning, this, _1, _2),
                    ros::VoidPtr(), &cb_queue_);

        srv_learn_face = nh.advertiseService(opt_srv_get_entity_info);

        // publish actionlib server
        as_ = new actionlib::SimpleActionServer<ed_perception::FaceLearningAction> (nh, "face_recognition/learn_face", false);
        as_->registerGoalCallback(boost::bind(&FaceRecognition::learning_as_goal, this));
        as_->registerPreemptCallback(boost::bind(&FaceRecognition::learning_as_preempt, this));

        as_->start();

        std::cout << "[" << module_name_ << "] " << "Face learning service available." << std::endl;
    }

    init_success_ = true;

    std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;
}


// ----------------------------------------------------------------------------------------------------

void FaceRecognition::loadConfig(const std::string& config_path) {


    // default values in case configure(...) is not called!
    module_name_ = "face_recognition";
    debug_mode_ = false;
    save_learned_faces_ = false;
    saved_faces_dir_ = "/faces_learned/";
    using_Eigen_ = false;
    using_Fisher_ = true;
    using_LBPH_ = true;
    using_histogram_ = true;
    enable_learning_service_ = true;
    eigen_treshold_ = 3500;
    fisher_treshold_ = 800;
    lbph_treshold_ = 45;
    recognition_treshold_ = 0.95;
    face_target_size_ = 150;
    face_vert_offset_ = 0.35;
    face_horiz_offset_ = 0.25;
    learning_mode_ = false;
    max_faces_learn_ = 10;

    // ---------------- INITIALIZATIONS ----------------

    trained_Eigen_ = false;
    trained_Fisher_ = false;
    trained_LBPH_ = false;

    last_label_ = 0;

    // resize to the maximum number of FaceRecognizers you're going to use
    models_.resize(3);
    models_[EIGEN] = cv::createEigenFaceRecognizer();
    models_[FISHER] = cv::createFisherFaceRecognizer();
    models_[LBPH] = cv::createLBPHFaceRecognizer();

    // ---------------- DEBUGGGGGGGG

    // Get the path to the CSV file and images
//    saved_faces_dir_ = (std::string)getenv("HOME") + "/faces_learned/";
//    std::string csv_file_path = saved_faces_dir_ + "face_models.cvs";

//    // Load faces from CSV file
//    if (!csv_file_path.empty()){
//        readCSV(csv_file_path, images_, labels_, labels_info_);
//        trainRecognizers(images_, labels_, models_);
//    }else {
//        std::cout << "[" << module_name_ << "] " << "No CSV faces file was loaded! Recognizers not trained" << std::endl;
//    }

//    lbph_treshold_ = 80;

//    init_success_ = true;

//    std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;

}


// ----------------------------------------------------------------------------------------------------


void FaceRecognition::process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const{

    ed::ErrorContext errc("Processing entity in FaceRecognition");

    const ed::EntityConstPtr& e = input.entity;
    tue::Configuration& config = output.data;

    output.type_update.setUnknownScore(1);

    if (!init_success_)
        return;

    if (enable_learning_service_){
        cb_queue_.callAvailable();
    }

    if (!isFaceFound(config.limitScope())){
//        std::cout << "[" << module_name_ << "] " << "Couldn't find a face, skipping recognition"<< std::endl;
        return;
    }

    // ---------- Prepare measurement ----------

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->lastMeasurement();
    if (!msr)
        return;

    // create a view
    rgbd::View view(*msr->image(), msr->image()->getRGBImage().cols);

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // crop it to match the view
    cv::Mat cropped_image(color_image(cv::Rect(0,0,view.getWidth(), view.getHeight())));

    // ----------------------- Process -----------------------

    // face recognition
    std::vector<FaceInfo> faces_detected;

    // color recognition, for clothes
    cv::Mat entity_histogram;

    // get location of faces, if there are none then return
    if (!getFacesInfo(config.limitScope(), faces_detected)){
        return;
    }

    // get color information from the entity config and creat a histogram with it
    getEntityHistogram(config.limitScope(), entity_histogram);

    // pre-process every face
    for (uint i=0 ; i<faces_detected.size() ; i++){
        // align, grayscale and resize face
        if (!alignFace(cropped_image, faces_detected[i].features, face_target_size_, face_horiz_offset_, face_vert_offset_, faces_detected[i].face_img))
            std::cout << "[" << module_name_ << "] " << "Could not align face!" << std::endl;
    }


    // ---------- Learning (if active)----------
    if (learning_mode_){
        std::cout << "[" << module_name_ << "] " << "In learning mode..." << std::endl;

        int largest_face_idx = 0;
        int biggest_area = 0;

        ed::ErrorContext errc("Learning mode started in FaceRecognition");

//        face_match_result = learning_name_;
//        face_confidence_match = 0;

        // get biggest face available
        for (uint i=0 ; i<faces_detected.size() ; i++){
            if (faces_detected[i].features.width * faces_detected[i].features.height > biggest_area){
                biggest_area = faces_detected[i].features.width * faces_detected[i].features.height;
                largest_face_idx = i;
            }
        }

        // true when learning is complete
        if (learnFace(learning_name_, last_label_, faces_detected[largest_face_idx].face_img, entity_histogram, n_faces_current_, images_, labels_, labels_info_)){
            ed_perception::FaceLearningResult learning_result_;

            // reset number of learned faces
            n_faces_current_ = 0;

            // update last label number
            last_label_++;

            // retrain FaceRecognizers
            trainRecognizers(images_, labels_, models_);

            // quit learning mode
            learning_mode_ = false;

            if (as_->isActive()){
                learning_result_.result_info = "Learning complete";
                as_->setSucceeded(learning_result_);
            }

            std::cout << "[" << module_name_ << "] " << "Learning complete!" << std::endl;

        }else{
            // update number of faces learned
            n_faces_current_ ++;

            if (as_->isActive()){
                ed_perception::FaceLearningFeedback learning_feedback_;
                learning_feedback_.faces_learned = n_faces_current_;
                as_->publishFeedback(learning_feedback_);
            }
        }
    }


    // ---------- Recognition ----------

    ed::ErrorContext errc2("Predicting face");

    for (uint i=0 ; i<faces_detected.size() ; i++){

        // perform recognition on each module
        if (using_Eigen_ && trained_Eigen_){
            models_[EIGEN]->predict(faces_detected[i].face_img,
                                    faces_detected[i].predicted_label[EIGEN],
                                    faces_detected[i].confidence[EIGEN]);
        }

        if (using_Fisher_ && trained_Fisher_){
            models_[FISHER]->predict(faces_detected[i].face_img,
                                     faces_detected[i].predicted_label[FISHER],
                                     faces_detected[i].confidence[FISHER]);
        }

        if (using_LBPH_ && trained_LBPH_){
            models_[LBPH]->predict(faces_detected[i].face_img,
                                   faces_detected[i].predicted_label[LBPH],
                                   faces_detected[i].confidence[LBPH]);
        }

        if (using_histogram_){
            matchHistograms(entity_histogram,
                            learned_histograms_,
                            faces_detected[i].predicted_label[HIST],
                            faces_detected[i].confidence[HIST]);
        }

        // fill labels with the predicted name, in case there was a prediction
        faces_detected[i].predicted_name[EIGEN] = faces_detected[i].predicted_label[EIGEN] > -1 ?
                    labels_info_.at(faces_detected[i].predicted_label[EIGEN]) : "";

        faces_detected[i].predicted_name[FISHER] = faces_detected[i].predicted_label[FISHER] > -1 ?
                    labels_info_.at(faces_detected[i].predicted_label[FISHER]) : "";

        faces_detected[i].predicted_name[LBPH] = faces_detected[i].predicted_label[LBPH] > -1 ?
                    labels_info_.at(faces_detected[i].predicted_label[LBPH]) : "";

        faces_detected[i].predicted_name[HIST] = faces_detected[i].predicted_label[HIST] > -1 ?
                    labels_info_.at(faces_detected[i].predicted_label[HIST]) : "";


        // match different classifications into a single result
        matchClassifications(faces_detected[i].predicted_name,
                             faces_detected[i].confidence,
                             faces_detected[i].face_match_result,
                             faces_detected[i].face_confidence_match);
    }

    // ----------------------- Assert results -----------------------

    // create group if it doesnt exist
    if (!config.readGroup("perception_result", tue::OPTIONAL))
    {
        config.writeGroup("perception_result");
    }

    config.writeGroup("face_recognizer");

    if (faces_detected.size() > 0){
        config.writeArray("face");

        for (uint i=0 ; i<faces_detected.size() ; i++){
            config.addArrayItem();

            // only publish a name if the error is bellow the treshold
            if (!faces_detected[i].face_match_result.empty() && faces_detected[i].face_confidence_match < recognition_treshold_){
                config.setValue("label", faces_detected[i].face_match_result);
                config.setValue("score", faces_detected[i].face_confidence_match);
                config.setValue("index", faces_detected[i].index);
            }else{
                config.setValue("label", "");
                config.setValue("score", 0);
                config.setValue("index", faces_detected[i].index);
            }

            if (using_Eigen_ && faces_detected[i].predicted_label[EIGEN] > -1){
                config.writeGroup("eigen");
                config.setValue("name", faces_detected[i].predicted_name[EIGEN]);
                config.setValue("score", faces_detected[i].confidence[EIGEN]);
                config.endGroup();  // close eigen group
            }

            if (using_Fisher_ && faces_detected[i].predicted_label[FISHER] > -1){
                config.writeGroup("fisher");
                config.setValue("name", faces_detected[i].predicted_name[FISHER]);
                config.setValue("score", faces_detected[i].confidence[FISHER]);
                config.endGroup();  // close fisher group
            }

            if (using_LBPH_ && faces_detected[i].predicted_label[LBPH] > -1){
                config.writeGroup("lbph");
                config.setValue("name", faces_detected[i].predicted_name[LBPH]);
                config.setValue("score", faces_detected[i].confidence[LBPH]);
                config.endGroup();  // close lbph group
            }

            config.endArrayItem();
        }

        config.endArray();
    }

    // set unknown to 1 because plugin only cassifies faces
    output.type_update.setUnknownScore(1);

    config.endGroup();  // close face_recognizer group
    config.endGroup();  // close perception_result group

    if (debug_mode_){
//        showDebugWindow(face_aligned,
//                        predicted_name,
//                        confidence,
//                        face_match_result,
//                        face_confidence_match);
    }
}


// ----------------------------------------------------------------------------------------------------


void FaceRecognition::getEntityHistogram(tue::Configuration config, cv::Mat& entity_histogram) const{

    std::string color;
    double amount;

    // initialize histogram
    entity_histogram = cv::Mat::zeros(1, ColorNameTable::NUM_COLORS, CV_32FC1);

    // crawl through the config
    if (!config.readGroup("perception_result")) return;
    if (!config.readGroup("color_matcher")) return;
    if (!config.readArray("colors")) return;

    while(config.nextArrayItem()){
        color = "";

        // get color info
        if (config.value("name", color) && config.value("value", amount)){
            // Iterate through all existing colors, Orange is 0, Black is 10, because...
            for (ColorNames::Color it_color = ColorNames::Orange; it_color <= ColorNames::Black; ++it_color)
            {
                // read the color and amount
                if (color.compare(colorToString(it_color)) == 0){
                    color = colorToString(it_color);
                    break;
                }
            }
        }

        if (color.empty())
            std::cout << "[" << module_name_ << "] " << "Error: Unmatched color name, not good!" << std::endl;

        // add color and amount to histogram
        entity_histogram.at<float>(colorToInt(color)) = amount;
    }

    config.endGroup();  // close color_matcher group
    config.endGroup();  // close perception_result group
}


// ----------------------------------------------------------------------------------------------------


bool FaceRecognition::learnFace(std::string person_name,
                                int person_label,
                                cv::Mat& face,
                                cv::Mat& histogram,
                                int n_face,
                                std::vector<cv::Mat>& face_images,
                                std::vector<int>& face_labels,
                                std::map<int, std::string>& faces_info) const{

    std::map <int, std::vector<cv::Mat> >::iterator find_it;

    std::cout << "[" << module_name_ << "] " << "Learning face: " << person_name << ", with label " << person_label
              << " (" << n_faces_current_ << "/" << max_faces_learn_ << ")" << std::endl;

    // add face and name to the DB
    face_images.push_back(face);
    face_labels.push_back(person_label);
    faces_info.insert(std::make_pair(person_label, person_name));

    // add histogram with cloths colors to the BD
    find_it = learned_histograms_.find(person_label);

    // add histogram to existing map entry, or create new
    if (find_it != learned_histograms_.end()){
        (find_it->second).push_back(histogram);
    }
    else{
        std::vector<cv::Mat> temp_vec;
        temp_vec.push_back(histogram);
        learned_histograms_.insert(std::pair<int, std::vector<cv::Mat> >(person_label, temp_vec));
    }

    if (save_learned_faces_){
        // -------------------- save faces for re-learning --------------------

        // create faces folder if it doesn't exist already
        std::string local_save_dir = saved_faces_dir_ + person_name + "/";
        boost::filesystem::path dir(local_save_dir);
        if( !(boost::filesystem::exists(dir)))
            boost::filesystem::create_directories(dir);

        // save image
        std::string file_name = person_name + "_" + ed::Entity::generateID().c_str();
        cv::imwrite(local_save_dir + file_name + ".pgm", face);


        // -------------------- update CSV file --------------------
        std::stringstream path_on_disk;
        path_on_disk << local_save_dir << file_name;

        // Add path to cvs file
        std::string cvs_file_name = saved_faces_dir_ + "face_models.cvs";
        std::fstream myfile;
        myfile.open (cvs_file_name.c_str(), std::fstream::in | std::fstream::out | std::fstream::app);
        myfile << local_save_dir << file_name << ".pgm" << ";" << person_label << ";" << person_name << "\n";
        myfile.close();
    }

    // if the number os faces learned for this user is more than max_faces_learn_, then its done
    return n_face >= max_faces_learn_;
}


// ----------------------------------------------------------------------------------------------------


void FaceRecognition::matchHistograms(cv::Mat& entity_histogram,
                                      std::map<int, std::vector<cv::Mat> >& learned_histograms,
                                      int& color_match_label,
                                      double& color_match_confidence) const{

    std::map<int, std::vector<cv::Mat> >::const_iterator map_it;
    std::vector<double> scores;
    double avg_score;
    double score;

    // iterate through all people learned
    for(map_it = learned_histograms.begin(); map_it != learned_histograms.end(); ++map_it){
        avg_score = 0;
        scores.clear();

//        std::cout << "[" << module_name_ << "] " << "For " << labels_info_.at(map_it->first) << std::endl;
        // iterate through each histogram from this person
        for(uint i=0 ; i< map_it->second.size() ; i++){
            score = cv::compareHist(entity_histogram, map_it->second[i], CV_COMP_CORREL);

            // sometimes the comprisson returns a negative score, not sure why but i dont like it
            if (score < 0)
                score = 0;

            scores.push_back(score);
//            std::cout << "[" << module_name_ << "] " << "\t: " << scores[i] << std::endl;
        }

        // calculate average
        for(uint i=0 ; i < scores.size() ; i++){
            avg_score += scores[i];
        }

        avg_score /= scores.size();
//        std::cout << "[" << module_name_ << "] " << "avg: " << avg_score << std::endl;

        if (color_match_confidence < avg_score){
            color_match_confidence = avg_score;
            color_match_label = map_it->first;
        }
    }
}


// ----------------------------------------------------------------------------------------------------


void FaceRecognition::matchClassifications(std::vector<std::string> classifications,
                                           std::vector<double> confidence,
                                           std::string& label_match,
                                           double& confidence_match) const{


    std::map <std::string, double> scores_positive;     // average scores for classifications bellow the thresholds
    std::map <std::string, double> scores_negative;     // average scores for classifications above the thresholds
    std::map <std::string, double>::iterator find_it;

    double norm_confidence;
    std::string recogizer_label;


    // go through all classifications, Eigen, Fisher and LBPH
    for (int i=0 ; i < classifications.size() ; i ++){

        // get classification name and calculate normalized confidence
        if (i == EIGEN && using_Eigen_ && !classifications[EIGEN].empty()){
            norm_confidence = confidence[EIGEN]/eigen_treshold_;
            recogizer_label = classifications[EIGEN];

        }else if (i == FISHER && using_Fisher_ && !classifications[FISHER].empty()){
            norm_confidence = confidence[FISHER]/fisher_treshold_;
            recogizer_label = classifications[FISHER];

        }else if (i == LBPH && using_LBPH_ && !classifications[LBPH].empty()){
            norm_confidence = confidence[LBPH]/lbph_treshold_;
            recogizer_label = classifications[LBPH];
        }else{
            continue;
        }

        // calculate final confidence for every classification, separated by threshold levels
        if(norm_confidence < 1.0){
            find_it = scores_positive.find(recogizer_label);

            if (find_it != scores_positive.end())
                scores_positive[recogizer_label] = (scores_positive[recogizer_label] + norm_confidence)/2;
            else
                scores_positive[recogizer_label] = norm_confidence;
        }else{
            find_it = scores_negative.find(recogizer_label);

            if (find_it != scores_negative.end())
                scores_negative[recogizer_label] = (scores_negative[recogizer_label] + norm_confidence)/2;
            else
                scores_negative[recogizer_label] = norm_confidence;
        }
    }

    // return best positive result if possible, otherwise best false result, or none
    if (!scores_positive.empty()){
        label_match = scores_positive.begin()->first;
        confidence_match = scores_positive.begin()->second;
    }else if (!scores_negative.empty()){
        label_match = scores_negative.begin()->first;
        confidence_match = scores_negative.begin()->second;
    }else{
        label_match = "";
        confidence_match = 0;
    }
}


// ----------------------------------------------------------------------------------------------------


bool FaceRecognition::srvStartLearning(const ed_perception::LearnPerson::Request& ros_req, ed_perception::LearnPerson::Response& ros_res){
    std::cout << "[" << module_name_ << "] " << "Service called, person name: " << ros_req.person_name << std::endl;

    if (!learning_mode_){

        std::cout << "[" << module_name_ << "] " << "Learning service, received new request." << std::endl;

        learning_name_ = ros_req.person_name;
        n_faces_current_ = 0;

        learning_mode_ = true;

//        ros_res.info = "Learning started";

//        return true;
    }else{
        ros_res.info = "Learning already in progress! Please wait.";

        return false;
    }
}


// ----------------------------------------------------------------------------------------------------


void FaceRecognition::learning_as_goal(){

    std::cout << "[" << module_name_ << "] " << "Learning service, received new goal." << std::endl;

    if (!learning_mode_){
        n_faces_current_ = 0;
        learning_mode_ = true;

        // accept the new goal and get the name
        learning_name_ = as_->acceptNewGoal()->person_name;;

        std::cout << "[" << module_name_ << "] " << "Learning person with name: " << learning_name_ << std::endl;
    }else{
        ed_perception::FaceLearningResult learning_result_;
        learning_result_.result_info = "Learning already in progress! Cannot take another request.";
        as_->setAborted(learning_result_);
    }
}


// ----------------------------------------------------------------------------------------------------


void FaceRecognition::learning_as_preempt(){
    std::cout << "[" << module_name_ << "] " << "" << std::endl;

    // set the action state to preempted
    as_->setPreempted();
}


// ----------------------------------------------------------------------------------------------------


void FaceRecognition::showDebugWindow(cv::Mat face_aligned,
                                      std::vector<std::string> predicted_name,
                                      std::vector<double> confidence,
                                      std::string face_match,
                                      double face_confidence) const{

    int key_press;

    cv::Scalar color_red (0, 0, 255);
    cv::Scalar color_yellow (0, 255, 255);
    cv::Scalar color_green (0, 204, 0);
    cv::Scalar color_gray (100, 100, 100);
    cv::Scalar color_white (255, 255, 255);

    cv::Scalar eigen_color;
    cv::Scalar fisher_color;
    cv::Scalar lbph_color;
    cv::Scalar hist_match_color;


    cv::Mat debug_display(cv::Size(face_target_size_ + 60, face_target_size_ + 135), CV_8UC1, cv::Scalar(0,0,0));
    cv::Mat debug_roi = debug_display(cv::Rect(30,0, face_target_size_, face_target_size_));
    face_aligned.copyTo(debug_roi);

    cvtColor(debug_display, debug_display, CV_GRAY2RGB);

    // set colors of the text depending on the score
    if (confidence[EIGEN]/eigen_treshold_ > 0.0 && confidence[EIGEN]/eigen_treshold_ <= 0.8)
        eigen_color = color_green;
    else if (confidence[EIGEN]/eigen_treshold_ > 0.8 && confidence[EIGEN]/eigen_treshold_ <= 1.0)
        eigen_color = color_yellow;
    else if (confidence[EIGEN]/eigen_treshold_ > 1.0)
        eigen_color = color_red;
    else
        eigen_color = color_gray;

    if (confidence[FISHER]/fisher_treshold_ > 0.0 && confidence[FISHER]/fisher_treshold_ <= 0.8)
        fisher_color = color_green;
    else if (confidence[FISHER]/fisher_treshold_ > 0.8 && confidence[FISHER]/fisher_treshold_ <= 1.0)
        fisher_color = color_yellow;
    else if (confidence[FISHER]/fisher_treshold_ > 1.0 )
        fisher_color = color_red;
    else
        fisher_color = color_gray;

    if (confidence[LBPH]/lbph_treshold_ > 0.0 && confidence[LBPH]/lbph_treshold_ <= 0.8)
        lbph_color = color_green;
    else if (confidence[LBPH]/lbph_treshold_ > 0.8 && confidence[LBPH]/lbph_treshold_ <= 1.0)
        lbph_color = color_yellow;
    else if (confidence[LBPH]/lbph_treshold_ > 1.0 )
        lbph_color = color_red;
    else
        lbph_color = color_gray;

    if (confidence[HIST] > 0.0 && confidence[HIST] <= 0.6)
        hist_match_color = color_red;
    else if (confidence[HIST] > 0.6 && confidence[HIST] <= 0.8)
        hist_match_color = color_yellow;
    else if (confidence[HIST] > 0.8)
        hist_match_color = color_green;
    else
        hist_match_color = color_gray;

    std::string info1("Eingen: " + predicted_name[EIGEN] + " (" + boost::str(boost::format("%.0f") % confidence[EIGEN]) + ")");
    std::string info2("Fisher: " + predicted_name[FISHER] + " (" + boost::str(boost::format("%.0f") % confidence[FISHER]) + ")");
    std::string info3("LBPH:  " + predicted_name[LBPH] + " (" + boost::str(boost::format("%.0f") % confidence[LBPH]) + ")");

    std::string info5("Color:  " + predicted_name[HIST] + " (" + boost::str(boost::format("%.2f") % confidence[HIST]) + ")");

    std::string info4("Match:  " + face_match + " (" + boost::str(boost::format("%.2f") % face_confidence) + ")");

    // draw text
    cv::putText(debug_display, info1, cv::Point(10 , face_target_size_ + 20), 1, 1.1, eigen_color, 1, CV_AA);
    cv::putText(debug_display, info2, cv::Point(10 , face_target_size_ + 40), 1, 1.1, fisher_color, 1, CV_AA);
    cv::putText(debug_display, info3, cv::Point(10 , face_target_size_ + 60), 1, 1.1, lbph_color, 1, CV_AA);

    cv::putText(debug_display, info5, cv::Point(10 , face_target_size_ + 90), 1, 1.1, hist_match_color, 1, CV_AA);

    cv::putText(debug_display, info4, cv::Point(10 , face_target_size_ + 120), 1, 1.1, color_white, 1, CV_AA);

    cv::imshow("Face Recognition debug", debug_display);
}


// ----------------------------------------------------------------------------------------------------


bool FaceRecognition::isFaceFound(tue::Configuration config) const{

    double score;
    std::string group_label = "face";

    if (!config.readGroup("perception_result", tue::OPTIONAL)){
//        std::cout << "Could not find perception_result group" << std::endl;
        return false;
    }

    if (!config.readGroup("face_detector", tue::OPTIONAL)){
//        std::cout << "could not find face_detector group" << std::endl;
        return false;
    }

    if (config.value("score", score, tue::OPTIONAL) && config.value("label", group_label, tue::OPTIONAL)){
//        std::cout << "Face detector score: " << score << std::endl;
        return score > 0;
    }
}


// ----------------------------------------------------------------------------------------------------


bool FaceRecognition::getFacesInfo(tue::Configuration config, std::vector<FaceInfo>& faces) const{

    int topX = 0, topY= 0, width= 0, height= 0, index = 0;

    if (!config.readGroup("perception_result", tue::OPTIONAL)) return false;
    if (!config.readGroup("face_detector", tue::OPTIONAL)) return false;
    if (!config.readArray("faces_front", tue::OPTIONAL)) return false;

    while(config.nextArrayItem()){
        // if all info is available, then save it
        if (config.value("x", topX) && config.value("y", topY) && config.value("height", height) && config.value("width", width) &&
                config.value("index", index)){

           faces.push_back(FaceInfo(index, cv::Rect(topX, topY, width, height)));
        }
    }

    if (faces.size() > 0)
        return true;
    else
        return false;
}


// ----------------------------------------------------------------------------------------------------


void FaceRecognition::trainRecognizers(std::vector<cv::Mat>& images, std::vector<int>& labels, std::vector<cv::Ptr<cv::FaceRecognizer> >& models) const{

    if (images.empty()){
        std::cout << "[" << module_name_ << "] " << "No images found. Unable to train Recognizers" << std::endl;
        return;
    }

    trained_Eigen_ = false;
    trained_LBPH_ = false;
    trained_Fisher_ = false;

    std::cout << "[" << module_name_ << "] " << "Training with " << images.size() << " images for " << labels_info_.size() << " different people." << std::endl;

    // train Eingen Faces
    if (using_Eigen_){
        models[EIGEN]->train(images, labels);
        std::cout << "[" << module_name_ << "] " << "Eigen Faces trained!" << std::endl;
        trained_Eigen_ = true;
    }

    // train LBP
    if (using_LBPH_){
        models[LBPH]->train(images, labels);
        std::cout << "[" << module_name_ << "] " << "LBPH Faces trained!" << std::endl;
        trained_LBPH_ = true;
    }

    // train Fisher Faces
    if(using_Fisher_){
        if(labels_info_.size() > 1){
            models[FISHER]->train(images, labels);
            std::cout << "[" << module_name_ << "] " << "Fisher Faces trained!" << std::endl;
            trained_Fisher_ = true;
        }else
            std::cout << "[" << module_name_ << "] " << "Could not train Fisher Faces, needs more than 1 class!" << std::endl;
    }
}


// ----------------------------------------------------------------------------------------------------


void FaceRecognition::readCSV( const std::string& filename,
                               std::vector<cv::Mat>& images,
                               std::vector<int>& labels,
                               std::map<int, std::string>& labelsInfo,
                               char separator) {

    std::ifstream csv(filename.c_str());

    if (!csv) {
        std::cout << "[" << module_name_ << "] " << "No valid CSV file was given, " << filename << std::endl;
    }

    std::string line;
    std::string path;
    std::string classlabel;
    std::string info;
    int unique_names=0;

    while (getline(csv, line)) {
        std::stringstream line_stream(line);
        path.clear();
        classlabel.clear();
        info.clear();

        getline(line_stream, path, separator);
        getline(line_stream, classlabel, separator);
        getline(line_stream, info, separator);

        if(!path.empty() && !classlabel.empty()) {
//            std::cout << "Processing " << path << std::endl;
            int label = atoi(classlabel.c_str());

            if(!info.empty()){
                if (labelsInfo.find(label) == labelsInfo.end()) unique_names ++;
                labelsInfo.insert(std::make_pair(label, info));
            }

            // 'path' can be file, dir or wildcard path
            std::string root(path.c_str());
            std::vector<std::string> files;
            cv::glob(root, files, true);

            for(std::vector<std::string>::const_iterator f = files.begin(); f != files.end(); ++f) {
//                std::cout << "\t reading from: " << *f << std::endl;
                cv::Mat img = cv::imread(*f, CV_LOAD_IMAGE_GRAYSCALE);
                static int w=-1, h=-1;
                static bool showSmallSizeWarning = true;

                if(w>0 && h>0 && (w!=img.cols || h!=img.rows)) std::cout << "\t* Warning: images should be of the same size!" << std::endl;

                if(showSmallSizeWarning && (img.cols<50 || img.rows<50)) {
                    std::cout << "* Warning: for better results images should be not smaller than 50x50!" << std::endl;
                    showSmallSizeWarning = false;
                }

                images.push_back(img);
                labels.push_back(label);
            }
        }
    }

    // update last label
    last_label_ = unique_names;
}


// ----------------------------------------------------------------------------------------------------


bool FaceRecognition::alignFace(cv::Mat origImg, cv::Rect faceLoc, int targetSize, float horizOffset, float vertOffset, cv::Mat& faceImg) const{
    std::vector<cv::Rect> leftEyeLoc;
    std::vector<cv::Rect> rightEyeLoc;
    cv::Point leftEye;
    cv::Point rightEye;
    cv::Point refOrigin;
    cv::Point topCorner;
    cv::Rect cropArea;
    int bestEyeLeft = 0;
    int bestEyeRight = 0;
    int horizOffsetEye;
    int vertOffsetEye;
    int dist;
    float scale;
    bool faceAligned = false;
    cv::Mat faceDetectGray;

    ed::ErrorContext errc("Aligning face");

    // create copy of the image for the haar detector and convert it to gray scale
    cvtColor(origImg(faceLoc), faceDetectGray, CV_BGR2GRAY);
    cv::equalizeHist(faceDetectGray, faceDetectGray);

    // detect eyes
    //leftEyeDetector_.detectMultiScale(faceDetectGray, leftEyeLoc, 1.1, 1, 0 | CV_HAAR_SCALE_IMAGE);
    //rightEyeDetector_.detectMultiScale(faceDetectGray, rightEyeLoc, 1.1, 1, 0 | CV_HAAR_SCALE_IMAGE);

    // find the first detected eye that is on the correct side of the face
    for (uint i = 0; i < leftEyeLoc.size(); i++){
        if (leftEyeLoc[i].x < faceDetectGray.cols / 2){
            bestEyeLeft = i;
            break;
        }
    }

    for (uint i = 0; i < rightEyeLoc.size(); i++){
        if (rightEyeLoc[i].x > faceDetectGray.cols / 2){
            bestEyeRight = i;
            break;
        }
    }

    // get center point for the eyes, if found
    if (leftEyeLoc.size() > 0 && rightEyeLoc.size() > 0){
        // get the center of the eye location
        leftEye = cv::Point(leftEyeLoc[bestEyeLeft].x + leftEyeLoc[bestEyeLeft].width / 2,
                leftEyeLoc[bestEyeLeft].y + leftEyeLoc[bestEyeLeft].height / 2);

        rightEye = cv::Point(rightEyeLoc[bestEyeRight].x + rightEyeLoc[bestEyeRight].width / 2,
                rightEyeLoc[bestEyeRight].y + rightEyeLoc[bestEyeRight].height / 2);
    }else{
        leftEye = cv::Point(0,0);
        rightEye = cv::Point(0,0);
    }

    // if the eyes are on the correct side of the face
    if (leftEye.x < faceDetectGray.rows / 2 && rightEye.x > faceDetectGray.rows / 2) {
        refOrigin = cv::Point(faceLoc.x, faceLoc.y);

        // calculate distance between the eyes in original scale
        dist = euclidDistance(leftEye, rightEye);

        if (dist > origImg.cols){
            std::cout << "[" << module_name_ << "] " << images_.size() << " Bad eye positions. Aborting alignement!" << std::endl;
            //leftEye.x, leftEye.y, rightEye.x, rightEye.y;
            return false;
        }

        // copy original color image to align it
        origImg.copyTo(faceImg);

        // draw circles on the eyes
//        circle(faceImg, leftEye + refOrigin, 2, cv::Scalar(0, 255, 0), 1);
//        circle(faceImg, rightEye + refOrigin, 2, cv::Scalar(0, 0, 255), 1);

        // determine the horizontal and vertical offeset for the left eye
        horizOffsetEye = floor(horizOffset * targetSize);
        vertOffsetEye = floor(vertOffset * targetSize);

        // determine the scale value so that the current distance between the eyes will cv::Match
        //	the one defined by the offsets
        scale = (targetSize - 2 * horizOffsetEye) / (float) dist;

        //rotate and resize the full image
        rotateFace(faceImg, faceImg, leftEye + refOrigin, rightEye + refOrigin, leftEye + refOrigin);
        resize(faceImg, faceImg, cv::Size(), scale, scale, cv::INTER_NEAREST);

        // top corner is the position of the left eye - offsets
        topCorner = cv::Point((leftEye.x + refOrigin.x) * scale, (leftEye.y + refOrigin.y) * scale)
                 - cv::Point (horizOffsetEye, vertOffsetEye);

        cropArea = cv::Rect(clipInt(topCorner.x, 0 , faceImg.cols - targetSize),
                        clipInt(topCorner.y, 0 , faceImg.rows - targetSize),
                        targetSize, targetSize);

        if (topCorner.x > faceImg.cols - targetSize || topCorner.y > faceImg.rows - targetSize)
            std::cout << "[" << module_name_ << "] " << " Top Corner had to be clipped, face ROI was too small" << std::endl;

        // crop the aligned image
        faceImg(cropArea).copyTo(faceImg);

        // eyes were corcv::Rectly detected
        faceAligned = true;
    }else{
        // if the eye detection failed, still focus more the original face ROI
        int topOffset;
        int size;

        // make the selected area square
        if (faceDetectGray.cols < faceDetectGray.rows){
            // offset of the new face rectangle compared to the original one
            topOffset = faceDetectGray.cols * 0.05;
            // zoom in on the face
            size = faceDetectGray.cols * 0.90;
        }else{
            // offset of the new face rectangle compared to the original one
            topOffset = faceDetectGray.rows * 0.05;
            // zoom in on the face
            size = faceDetectGray.rows * 0.90;
        }

        // crop the face, without going over the image
        faceDetectGray(cv::Rect(topOffset, topOffset*2,
                     clipInt(size, 1, faceDetectGray.cols - topOffset),
                     clipInt(size, 1, faceDetectGray.rows - topOffset*2))).copyTo(faceDetectGray);

        resize(faceDetectGray, faceImg, cv::Size(targetSize, targetSize), 0, 0, cv::INTER_NEAREST);

        faceAligned = true;
    }

    return faceAligned;
}

// ----------------------------------------------------------------------------------------------------

int FaceRecognition::clipInt(int val, int min, int max) const{
    return val <= min ? min : val >= max ? max : val;
}

// ----------------------------------------------------------------------------------------------------

int FaceRecognition::euclidDistance(cv::Point p1, cv::Point p2) const{
    float dx;
    float dy;
    dx = p2.x - p1.x;
    dy = p2.y - p1.y;
    return floor(sqrt(dx * dx + dy * dy));
}

// ----------------------------------------------------------------------------------------------------

void FaceRecognition::rotateFace(cv::Mat faceImg, cv::Mat& rotatedImg, cv::Point leftEye, cv::Point rightEye, cv::Point center) const{
    double mod1, mod2, innerp, angle;
    cv::Point p3 (rightEye.x, leftEye.y);
    cv::Mat rotMat;

    // module of the line between right eye and left eye
    mod1 = sqrt(pow(rightEye.y - leftEye.y, 2) + pow(rightEye.x - leftEye.x, 2));
    // module of the line between the left eye and a cv::Point in the horizontal line passing bye the left eye
    mod2 = sqrt(pow(p3.y - leftEye.y, 2) + pow(rightEye.x - leftEye.x, 2));
    // calculate inner product
    innerp = (rightEye.x - leftEye.x) * (p3.x - leftEye.x) + (rightEye.y - leftEye.y) * (p3.y - leftEye.y);
    // calculate angle and convert it to degrees
    angle = acos(innerp / (mod1 * mod2)) * 180 / M_PI;

    // determine if its a positive or negative rotation
    if (leftEye.y > rightEye.y)
        angle *= -1;

    // get rotation cv::Matrix and apply it to the image
    rotMat = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::warpAffine(faceImg, rotatedImg, rotMat, faceImg.size());
}

ED_REGISTER_PERCEPTION_MODULE(FaceRecognition)

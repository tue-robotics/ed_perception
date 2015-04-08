/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#include "face_detector.h"

#include "ed/measurement.h"
#include <ed/entity.h>
#include <ed/error_context.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <boost/filesystem.hpp>

// ----------------------------------------------------------------------------------------------------

FaceDetector::FaceDetector() :
    ed::perception::Module("face_detector"),
    init_success_(false)
{

}


// ----------------------------------------------------------------------------------------------------

FaceDetector::~FaceDetector()
{
    // destroy the debug window
    if (debug_mode_) {
        cv::destroyWindow("Face Detector Output");
    }
}

// ----------------------------------------------------------------------------------------------------

void FaceDetector::configure(tue::Configuration config) {

    if (!config.value("cascade_front_files_path", cascade_front_files_path_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'cascade_front_files_path' not found. Using default: " << cascade_front_files_path_ << std::endl;

    if (!config.value("cascade_profile_front_path", cascade_profile_files_path_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'cascade_profile_front_path' not found. Using default: " << cascade_profile_files_path_ << std::endl;

    if (!config.value("debug_mode", debug_mode_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'debug_mode' not found. Using default: " << debug_mode_ << std::endl;

    if (!config.value("debug_folder", debug_folder_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'debug_folder' not found. Using default: " << debug_folder_ << std::endl;

    if (!config.value("classifier_front_scale_factor", classifier_front_scale_factor_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'classifier_front_scale_factor' not found. Using default: " << classifier_front_scale_factor_ << std::endl;

    if (!config.value("classifier_front_min_neighbours", classifier_front_min_neighbours_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'classifier_front_min_neighbours' not found. Using default: " << classifier_front_min_neighbours_ << std::endl;

    if (!config.value("classifier_profile_scale_factor", classifier_profile_scale_factor_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'classifier_profile_scale_factor' not found. Using default: " << classifier_profile_scale_factor_ << std::endl;

    if (!config.value("classifier_profile_min_neighbours", classifier_profile_min_neighbours_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'classifier_profile_min_neighbours' not found. Using default: " << classifier_profile_min_neighbours_ << std::endl;

    if (!config.value("type_positive_score", type_positive_score_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'type_positive_score' not found. Using default: " << type_positive_score_ << std::endl;

    if (!config.value("type_negative_score", type_negative_score_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'type_negative_score' not found. Using default: " << type_negative_score_ << std::endl;

    cascade_front_files_path_ = module_path_ + cascade_front_files_path_;
    cascade_profile_files_path_ = module_path_ + cascade_profile_files_path_;

    if (debug_mode_){
        // clean the debug folder if debugging is active
        try {
            boost::filesystem::path dir(debug_folder_);
            boost::filesystem::remove_all(dir);
            boost::filesystem::create_directories(dir);
        } catch(const boost::filesystem::filesystem_error& e){
           if(e.code() == boost::system::errc::permission_denied)
               std::cout << "[" << module_name_ << "] " << "boost::filesystem permission denied" << std::endl;
           else
               std::cout << "[" << module_name_ << "] " << "boost::filesystem failed with error: " << e.code().message() << std::endl;
        }

        // create debug window
        cv::namedWindow("Face Detector Output", CV_WINDOW_AUTOSIZE);
    }


    // check if the cascade file exist so they can be loaded later
    if (!boost::filesystem::exists(cascade_front_files_path_) || !boost::filesystem::exists(cascade_profile_files_path_)){
        init_success_ = false;
        std::cout << "[" << module_name_ << "] " << "Couldn't find cascade files for detection (" << cascade_profile_files_path_
                  <<  "). Face dection will not work!" << std::endl;
    }else{
        std::cout << "[" << module_name_ << "] " << "Face Detection cascade files found." << std::endl;

        init_success_ = true;
        std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;
    }
}


// ----------------------------------------------------------------------------------------------------


void FaceDetector::loadConfig(const std::string& config_path) {

    module_name_ = "face_detector";
    module_path_ = config_path;

    // default values in case configure(...) is not called!
    cascade_front_files_path_ = "/cascade_classifiers/haarcascade_frontalface_alt_tree.xml";
    cascade_profile_files_path_ = "/cascade_classifiers/haarcascade_profileface.xml";
    debug_mode_ = false;
    classifier_front_scale_factor_ = 1.2;
    classifier_front_min_neighbours_ = 3;
    classif_front_min_size_ = cv::Size(20,20);
    classifier_profile_scale_factor_= 1.2;
    classifier_profile_min_neighbours_ = 3;
    classif_profile_min_size_ = cv::Size(20,20);
    debug_folder_ = "/tmp/face_detector/";
    type_positive_score_ = 0.9;
    type_negative_score_ = 0.4;

    shared_methods = SharedMethods();


    // --------------DEBUGGGGGGGG
//    cascade_front_files_path_ = module_path_ + cascade_front_files_path_;
//    cascade_profile_files_path_ = module_path_ + cascade_profile_files_path_;

//    // check if the cascade file exist so they can be loaded later
//    if (!boost::filesystem::exists(cascade_front_files_path_) || !boost::filesystem::exists(cascade_profile_files_path_)){
//        init_success_ = false;
//        std::cout << "[" << module_name_ << "] " << "Couldn't find cascade files for detection (" << cascade_profile_files_path_
//                  <<  "). Face dection will not work!" << std::endl;
//    }else{
//        init_success_ = true;
//        std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;
//    }
}


// ----------------------------------------------------------------------------------------------------


void FaceDetector::process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const
{
    ed::ErrorContext errc("Processing entity in FaceDetector");

    const ed::EntityConstPtr& e = input.entity;
    tue::Configuration& result = output.data;

    if (!init_success_)
        return;

    // ---------- Prepare measurement ----------

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->lastMeasurement();
    if (!msr)
        return;

    std::vector<cv::Rect> faces_front;
    std::vector<cv::Rect> faces_profile;
    int min_x, max_x, min_y, max_y;
    int face_counter = 0;


    // create a view
    rgbd::View view(*msr->image(), msr->image()->getRGBImage().cols);

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // get color image
    const cv::Mat& depth_image = msr->image()->getDepthImage();

    // crop it to match the view
    cv::Mat cropped_image(color_image(cv::Rect(0,0,view.getWidth(), view.getHeight())));

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

    cv::Rect bouding_box (min_x, min_y, max_x - min_x, max_y - min_y);

    // ----------------------- Process and Assert results -----------------------

    // create group if it doesnt exist
    if (!result.readGroup("perception_result", tue::OPTIONAL))
    {
        result.writeGroup("perception_result");
    }

    result.writeGroup("face_detector");

    // Detect faces in the measurment and assert the results
    if(DetectFaces(cropped_image(bouding_box), faces_front, faces_profile)){
        geo::Vector3 projection;
        geo::Vector3 point_map;

        // if front faces were detected
        if (faces_front.size() > 0){
            result.writeArray("faces_front");
            for (uint j = 0; j < faces_front.size(); j++) {

                result.addArrayItem();

                result.setValue("index", face_counter);

                faces_front[j].x += min_x;
                faces_front[j].y += min_y;

                // add 2D location of the face
                result.setValue("x", faces_front[j].x);
                result.setValue("y", faces_front[j].y);
                result.setValue("width", faces_front[j].width);
                result.setValue("height", faces_front[j].height);

                // calculate the center point of the face
                cv::Point2i p_2d(faces_front[j].x + faces_front[j].width/2,
                                 faces_front[j].y + faces_front[j].height/2);

                cv::Mat face_area = depth_image(faces_front[j]);
                float avg_depth = shared_methods.getAverageDepth(face_area);

                if (avg_depth == 0){
                    std::cout << "[" << module_name_ << "] " << "Could not calculate face's average depth. Map coordinates might be incorrect!" << std::endl;
                }

                projection = view.getRasterizer().project2Dto3D(p_2d.x, p_2d.y) * avg_depth;
                point_map = msr->sensorPose() * projection;

                // add 3D location of the face
                result.setValue("map_x", point_map.x);
                result.setValue("map_y", point_map.y);
                result.setValue("map_z", point_map.z);

                result.endArrayItem();
                face_counter++;
            }
            result.endArray();
        }

        // if profile faces were detected
        if (faces_profile.size() > 0){
            result.writeArray("faces_profile");
            for (uint j = 0; j < faces_profile.size(); j++) {

                faces_profile[j].x += min_x;
                faces_profile[j].y += min_y;

                result.addArrayItem();

                result.setValue("index", face_counter);

                // add 2D location of the face
                result.setValue("x", faces_profile[j].x);
                result.setValue("y", faces_profile[j].y);
                result.setValue("width", faces_profile[j].width);
                result.setValue("height", faces_profile[j].height);

                // calculate the center point of the face
                cv::Point2i p_2d(faces_profile[j].x + faces_profile[j].width/2,
                                 faces_profile[j].y + faces_profile[j].height/2);

                cv::Mat face_area = depth_image(faces_profile[j]);
                float avg_depth = shared_methods.getAverageDepth(face_area);

                if (avg_depth == 0){
                    std::cout << "[" << module_name_ << "] " << "Could not calculate face's average depth. Map coordinates might be incorrect!" << std::endl;
                }

                projection = view.getRasterizer().project2Dto3D(p_2d.x, p_2d.y) * avg_depth;
                point_map = msr->sensorPose() * projection;

                // add 3D location of the face
                result.setValue("map_x", point_map.x);
                result.setValue("map_y", point_map.y);
                result.setValue("map_z", point_map.z);

                result.endArrayItem();
                face_counter++;
            }
            result.endArray();
        }

        if (faces_front.size() + faces_profile.size() > 1){
            result.setValue("label", "multiple_faces");
        }
        else{
            result.setValue("label", "face");
        }

        output.type_update.setScore("human", type_positive_score_);
        result.setValue("score", type_positive_score_);

    }else{
        // no faces detected
        result.setValue("label", "face");
        result.setValue("score", type_negative_score_);

        output.type_update.setScore("human", type_negative_score_);
    }

    result.endGroup();  // close face_detector group
    result.endGroup();  // close perception_result group
}


// ----------------------------------------------------------------------------------------------------


bool FaceDetector::DetectFaces(const cv::Mat& cropped_img,
                               std::vector<cv::Rect>& faces_front,
                               std::vector<cv::Rect>& faces_profile) const{

    cv::Mat cascade_img;
    std::vector<cv::Rect>::iterator face_it;

    // using locally created classifiers because opencv does not support threading
    cv::CascadeClassifier classifier_front_local;
    cv::CascadeClassifier classifier_profile_local;

    // create a copy of the image
    cropped_img.copyTo(cascade_img);

    // increase contrast of the image
    normalize(cascade_img, cascade_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // load training files for frontal classifier
    if (!classifier_front_local.load(cascade_front_files_path_)) {
        std::cout << "[" << module_name_ << "] " << "Unable to load front haar cascade files ("
                  << cascade_front_files_path_ << ")" << std::endl;

        return false;
    }

    // detect frontal faces
    classifier_front_local.detectMultiScale(cascade_img,
                                            faces_front,
                                            classifier_front_scale_factor_,
                                            classifier_front_min_neighbours_,
                                            0|CV_HAAR_SCALE_IMAGE,
                                            classif_front_min_size_);

    // discard face if its not close to the top of the region, false positive
    face_it = faces_front.begin();
    for ( ; face_it != faces_front.end(); ) {
        // allowed area is the full width and three times the size of the detected face
        cv::Rect allowed_area (0, 0, cropped_img.cols, face_it->height * 3);

        // test if the rectangles intersect
        if ( !(allowed_area & *face_it).area()) {
            face_it = faces_front.erase(face_it);
        }else
            ++face_it;
    }

    // only search profile faces if the frontal face detection failed
    if (faces_front.empty()){
        // load training files for profile classifier
        if (!classifier_profile_local.load(cascade_profile_files_path_)){
            std::cout << "[" << module_name_ << "] " << "Unable to load profile haar cascade files ("
                      << cascade_profile_files_path_ << ")" << std::endl;

            return false;
        }

        classifier_profile_local.detectMultiScale(cascade_img,
                                                  faces_profile,
                                                  classifier_profile_scale_factor_,
                                                  classifier_profile_min_neighbours_,
                                                  0|CV_HAAR_SCALE_IMAGE,
                                                  classif_profile_min_size_);

        // discard face if its not close to the top of the region, false positive
        face_it = faces_profile.begin();
        for ( ; face_it != faces_profile.end(); ) {
            // allowed area is the full width and three times the size of the detected face
            cv::Rect allowed_area (0, 0, cropped_img.cols, face_it->height * 3);

            // test if the rectangles intersect
            if ( !(allowed_area & *face_it).area()) {
                face_it = faces_profile.erase(face_it);
            }else
                ++face_it;
        }
    }


    // if debug mode is active and faces were found
    if (debug_mode_){
        cv::Mat debugImg(cropped_img);

        for (uint j = 0; j < faces_front.size(); j++)
            cv::rectangle(debugImg, faces_front[j], cv::Scalar(0, 255, 0), 2, CV_AA);

        for (uint j = 0; j < faces_profile.size(); j++)
            cv::rectangle(debugImg, faces_profile[j], cv::Scalar(0, 0, 255), 2, CV_AA);


        cv::imwrite(debug_folder_ + ed::Entity::generateID().c_str() + "_face_detector.png", debugImg);
        cv::imshow("Face Detector Output", debugImg);
    }

    // return true if a face was found
    return (!faces_front.empty() || !faces_profile.empty());
}


// ----------------------------------------------------------------------------------------------------


void FaceDetector::OptimizeContourHull(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const{

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


void FaceDetector::OptimizeContourBlur(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const{

    mask_orig.copyTo(mask_optimized);

    // blur the contour, also expands it a bit
    for (uint i = 6; i < 18; i = i + 2){
        cv::blur(mask_optimized, mask_optimized, cv::Size( i, i ), cv::Point(-1,-1) );
    }

    cv::threshold(mask_optimized, mask_optimized, 50, 255, CV_THRESH_BINARY);
}


// ----------------------------------------------------------------------------------------------------


int FaceDetector::ClipInt(int val, int min, int max) const{
    return val <= min ? min : val >= max ? max : val;
}

ED_REGISTER_PERCEPTION_MODULE(FaceDetector)

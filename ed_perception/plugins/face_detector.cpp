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

#include "shared_methods.h"

// ----------------------------------------------------------------------------------------------------

FaceDetector::FaceDetector() : ed::perception::Module("face_detector")
{
    this->registerPropertyServed("type");
    this->registerPropertyServed("name");
}


// ----------------------------------------------------------------------------------------------------

FaceDetector::~FaceDetector()
{
    // destroy the debug window
    if (debug_mode_)
    {
        cv::destroyWindow("Face Detector Output");
    }
}

// ----------------------------------------------------------------------------------------------------

void FaceDetector::configureClassification(tue::Configuration config)
{
    // default values in case configure(...) is not called!
    debug_mode_ = false;
    classifier_front_scale_factor_ = 1.2;
    classifier_front_min_neighbours_ = 3;
    classif_front_min_size_ = cv::Size(20,20);
    classifier_profile_scale_factor_= 1.2;
    classifier_profile_min_neighbours_ = 3;
    classif_profile_min_size_ = cv::Size(20,20);
    debug_folder_ = "/tmp/face_detector/";

    // load training files for frontal classifier
    std::string cascade_front_files_path_;
    if (config.value("cascade_front_files_path", cascade_front_files_path_))
    {
        if (!classifier_front_.load(cascade_front_files_path_))
            config.addError("Unable to load front haar cascade files (" + cascade_front_files_path_ + ")");
    }

    // load training files for profile classifier
    std::string cascade_profile_files_path_;
    if (config.value("cascade_profile_front_path", cascade_profile_files_path_))
    {
        if (!classifier_profile_.load(cascade_profile_files_path_))
            config.addError("Unable to load profile haar cascade files (" + cascade_profile_files_path_ + ")");
    }

    if (config.hasError())
        return;

    if (!config.value("debug_mode", debug_mode_, tue::OPTIONAL))
        ed::log::info() << "Parameter 'debug_mode' not found. Using default: " << debug_mode_ << std::endl;

    if (!config.value("debug_folder", debug_folder_, tue::OPTIONAL))
        ed::log::info() << "Parameter 'debug_folder' not found. Using default: " << debug_folder_ << std::endl;

    if (!config.value("classifier_front_scale_factor", classifier_front_scale_factor_, tue::OPTIONAL))
        ed::log::info() << "Parameter 'classifier_front_scale_factor' not found. Using default: " << classifier_front_scale_factor_ << std::endl;

    if (!config.value("classifier_front_min_neighbours", classifier_front_min_neighbours_, tue::OPTIONAL))
        ed::log::info() << "Parameter 'classifier_front_min_neighbours' not found. Using default: " << classifier_front_min_neighbours_ << std::endl;

    if (!config.value("classifier_profile_scale_factor", classifier_profile_scale_factor_, tue::OPTIONAL))
        ed::log::info() << "Parameter 'classifier_profile_scale_factor' not found. Using default: " << classifier_profile_scale_factor_ << std::endl;

    if (!config.value("classifier_profile_min_neighbours", classifier_profile_min_neighbours_, tue::OPTIONAL))
        ed::log::info() << "Parameter 'classifier_profile_min_neighbours' not found. Using default: " << classifier_profile_min_neighbours_ << std::endl;

    if (debug_mode_)
    {
        // clean the debug folder if debugging is active
        try {
            boost::filesystem::path dir(debug_folder_);
            boost::filesystem::remove_all(dir);
            boost::filesystem::create_directories(dir);
        }
        catch(const boost::filesystem::filesystem_error& e)
        {
            if(e.code() == boost::system::errc::permission_denied)
                ed::log::error() << "boost::filesystem permission denied" << std::endl;
            else
                ed::log::error() << "boost::filesystem failed with error: " << e.code().message() << std::endl;
        }

        // create debug window
        cv::namedWindow("Face Detector Output", CV_WINDOW_AUTOSIZE);
    }
}

// ----------------------------------------------------------------------------------------------------

void FaceDetector::classify(const ed::Entity& e, const std::string& property, const ed::perception::CategoricalDistribution& prior,
                            ed::perception::ClassificationOutput& output) const
{
    if (property != "type" && property != "name")
        return;

    // If we already know that this is not going to be a human, skip face detection altogether
    double prior_human;
    if (prior.getScore("human", prior_human) && prior_human == 0)
        return;

    // ---------- Prepare measurement ----------

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e.lastMeasurement();

    if (!msr)
        return;

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // get depth image
    const cv::Mat& depth_image = msr->image()->getDepthImage();

    // Mask color image
    cv::Rect rgb_roi;
    cv::Mat color_image_masked = ed::perception::maskImage(color_image, msr->imageMask(), rgb_roi);

    // ---------- Detect faces ----------

    std::vector<cv::Rect> faces_front;
    std::vector<cv::Rect> faces_profile;

    // Detect faces in the measurment and assert the results
    if (DetectFaces(color_image_masked(rgb_roi), faces_front, faces_profile))
    {
        // write face information to config if a frontal face was found
        int face_counter = 0;
        if (faces_front.size() > 0)
        {
            output.data.writeArray("faces_front");
            writeFaceDetectionResult(*msr, rgb_roi, faces_front, face_counter, output.data);
            output.data.endArray();
        }

        // write face information to config if a profile face was found
        if (faces_profile.size() > 0)
        {
            output.data.writeArray("faces_profile");
            writeFaceDetectionResult(*msr, rgb_roi, faces_profile, face_counter, output.data);
            output.data.endArray();
        }

        if (property == "type")
        {
            output.likelihood.setScore("human", 1);
        }
        else if (property == "name")
        {
            if (!faces_front.empty())
            {
                recognizeFace(color_image_masked(rgb_roi), faces_front[0], output);
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void FaceDetector::addTrainingInstance(const ed::Entity& e, const std::string& property, const std::string& value)
{
    std::cout << "FaceDetector::addTrainingInstance" << std::endl;

    if (property != "name")
        return;

    face_data_.insert(value);
}

// ----------------------------------------------------------------------------------------------------

bool FaceDetector::DetectFaces(const cv::Mat& cropped_img,
                               std::vector<cv::Rect>& faces_front,
                               std::vector<cv::Rect>& faces_profile) const{

    cv::Mat cascade_img;
    std::vector<cv::Rect>::iterator face_it;

    // create a copy of the image
    cropped_img.copyTo(cascade_img);

    // increase contrast of the image
    normalize(cascade_img, cascade_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);



    // detect frontal faces
    classifier_front_.detectMultiScale(cascade_img,
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
    if (faces_front.empty())
    {
        classifier_profile_.detectMultiScale(cascade_img,
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
    if (debug_mode_)
    {
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

void FaceDetector::recognizeFace(const cv::Mat& image, const cv::Rect& roi, ed::perception::ClassificationOutput& output) const
{
    if (face_data_.empty())
        return;

//    cv::Mat img = image.clone();
//    cv::rectangle(img, roi, cv::Scalar(0, 0, 255), 2);
//    cv::imshow("person", img);
//    cv::imshow("face_roi", image(roi));
//    cv::waitKey();

    // TODO: replace this with real face recognition!

    output.likelihood.setScore(*face_data_.begin(), 1);
}

// ----------------------------------------------------------------------------------------------------

void FaceDetector::writeFaceDetectionResult(const ed::Measurement& msr, const cv::Rect& rgb_roi,
                                            const std::vector<cv::Rect>& rgb_face_rois,
                                            int& face_counter, tue::Configuration& result) const
{
    // get color image
    const cv::Mat& color_image = msr.image()->getRGBImage();

    // get color image
    const cv::Mat& depth_image = msr.image()->getDepthImage();

    // Calculate size factor between depth and rgb images
    double f_depth_rgb = (double)depth_image.cols / color_image.cols;

    // Create depth view
    rgbd::View depth_view(*msr.image(), depth_image.cols);

    for (uint j = 0; j < rgb_face_rois.size(); j++)
    {
        cv::Rect rgb_face_roi = rgb_face_rois[j];
        rgb_face_roi.x += rgb_roi.x;
        rgb_face_roi.y += rgb_roi.y;

        if (debug_mode_)
            ed::perception::saveDebugImage("face_detector-rgb", color_image(rgb_face_roi));

        result.addArrayItem();

        result.setValue("index", face_counter);

        // add 2D location of the face
        result.setValue("x", rgb_face_roi.x);
        result.setValue("y", rgb_face_roi.y);
        result.setValue("width", rgb_face_roi.width);
        result.setValue("height", rgb_face_roi.height);

        // Compute face roi for depth image
        cv::Rect depth_face_roi(f_depth_rgb * rgb_face_roi.x, f_depth_rgb * rgb_face_roi.y,
                                f_depth_rgb * rgb_face_roi.width, f_depth_rgb * rgb_face_roi.height);

        if (debug_mode_)
            ed::perception::saveDebugImage("face_detector-depth", depth_image(depth_face_roi));

        cv::Mat face_area = depth_image(depth_face_roi);
        float avg_depth = ed::perception::getMedianDepth(face_area);

        if (avg_depth > 0)
        {
            // calculate the center point of the face
            cv::Point2i p_2d(depth_face_roi.x + depth_face_roi.width/2,
                             depth_face_roi.y + depth_face_roi.height/2);

            geo::Vector3 projection = depth_view.getRasterizer().project2Dto3D(p_2d.x, p_2d.y) * avg_depth;
            geo::Vector3 point_map = msr.sensorPose() * projection;

            // add 3D location of the face
            result.setValue("map_x", point_map.x);
            result.setValue("map_y", point_map.y);
            result.setValue("map_z", point_map.z);

        }
        else
        {
            std::cout << "[ED FACE DETECTOR] Could not calculate face's average depth. Map coordinates might be incorrect!" << std::endl;
        }

        result.endArrayItem();
        face_counter++;
    }
}

ED_REGISTER_PERCEPTION_MODULE(FaceDetector)

#include "face_recognizer.h"

#include <ros/package.h>
#include <opencv2/imgproc/imgproc.hpp>

// ----------------------------------------------------------------------------------------------------

FaceRecognizer::FaceRecognizer()
{
}

// ----------------------------------------------------------------------------------------------------

FaceRecognizer::~FaceRecognizer()
{
}

// ----------------------------------------------------------------------------------------------------

bool FaceRecognizer::initialize()
{
    // Load face classifier model from 'ed_perception_models' package
    std::string data_path = ros::package::getPath("ed_perception_models") + "/general/face/";
    face_detector_.load(data_path + "haarcascade_frontalface_alt_tree.xml");
}

// ----------------------------------------------------------------------------------------------------

bool FaceRecognizer::train(const cv::Mat& rgb_image, const std::string& name)
{
    // - - - - - - - - - - - - - - - - - - - - -
    // TODO: replace with training face

    // For now: simply remember the name
    names_.insert(name);

    // - - - - - - - - - - - - - - - - - - - - -

    return false;
}

// ----------------------------------------------------------------------------------------------------

void FaceRecognizer::find(const cv::Mat& rgb_image, std::vector<FaceRecognitionResult>& detections)
{
    // Convert image to grayscale
    cv::Mat image_grayscale;
    cv::cvtColor(rgb_image, image_grayscale, CV_BGR2GRAY);

    // Equalize histogram
    cv::equalizeHist( image_grayscale, image_grayscale );

    // Detect faces. Returns region of interests of the faces
    std::vector<cv::Rect> face_rois;
    face_detector_.detectMultiScale(image_grayscale, face_rois, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

    // For each roi: try to estimate which person the face belongs to, the age, the gender and the body pose
    detections.resize(face_rois.size());
    for(unsigned int i = 0; i < face_rois.size(); ++i)
    {
        FaceRecognitionResult& det = detections[i];
        det.rgb_roi = face_rois[i];

        // - - - - - - - - - - - - - - - - - - - - -
        // TODO: replace with face recognition

        // For now: simple dummy which always returns the first learned name
        if (!names_.empty())
            det.name = *names_.begin();

        // - - - - - - - - - - - - - - - - - - - - -
    }
}


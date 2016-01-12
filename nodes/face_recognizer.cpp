#include "face_recognizer.h"

// ----------------------------------------------------------------------------------------------------

FaceRecognizer::FaceRecognizer()
{
}

// ----------------------------------------------------------------------------------------------------

FaceRecognizer::~FaceRecognizer()
{
}

// ----------------------------------------------------------------------------------------------------

bool FaceRecognizer::train(const cv::Mat& rgb_image, const std::string& name)
{
    return false;
}

// ----------------------------------------------------------------------------------------------------

bool FaceRecognizer::find(const cv::Mat& rgb_image, const std::string& name, cv::Rect* roi) const
{
    return false;
}


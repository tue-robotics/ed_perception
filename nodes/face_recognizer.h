#ifndef _FACE_RECOGNIZER_H_
#define _FACE_RECOGNIZER_H_

#include <opencv2/core/core.hpp>

class FaceRecognizer
{

public:

    FaceRecognizer();

    ~FaceRecognizer();

    bool train(const cv::Mat& rgb_image, const std::string& name);

    bool find(const cv::Mat& rgb_image, const std::string& name, cv::Rect* roi) const;

private:

};

#endif

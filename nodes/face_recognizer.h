#ifndef _FACE_RECOGNIZER_H_
#define _FACE_RECOGNIZER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <set>
#include <tue/config/configuration.h>

// ----------------------------------------------------------------------------------------------------

enum Gender
{
    MALE,
    FEMALE
};

// ----------------------------------------------------------------------------------------------------

struct FaceRecognitionResult
{
    FaceRecognitionResult() : age(-1), name_score(0) {}

    // region of interest in the RGB image
    cv::Rect rgb_roi;

    // estimated name of the person
    std::string name;

    // the higher, the more probable 'name' is correct
    double name_score;

    // estimated gender
    Gender gender;

    // estimated age
    double age;

    // pose / stance of the person: standing, sitting, waving, etc
    std::string body_pose;
};

// ----------------------------------------------------------------------------------------------------

class FaceRecognizer
{

public:

    FaceRecognizer();

    ~FaceRecognizer();

    // Initialize detector / recognizer
    void configure(tue::Configuration& config);

    // Detect a face in the given image, and train it as being 'name'
    bool train(const cv::Mat& rgb_image, const std::string& name);

    // Detect faces in the given image and esimate their name, age, etc.
    void find(const cv::Mat& rgb_image, std::vector<FaceRecognitionResult>& detections);

    // Clear stored names
    void clear();

private:

    // default OpenCV face detector
    cv::CascadeClassifier face_detector_;

    // - - - - - - - - - - - - - - - - - - - - -
    // TODO: replace with person / face models per person name

    std::set<std::string> names_;

    // - - - - - - - - - - - - - - - - - - - - -


};

#endif

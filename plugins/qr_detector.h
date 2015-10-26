#ifndef qr_detector_h_
#define qr_detector_h_

#include <ed/perception/module.h>

class QRDetector : public ed::perception::Module
{

public:

    QRDetector();

    virtual ~QRDetector();

    void configure(tue::Configuration config);

    void loadModel(const std::string& model_name, const std::string& model_path);

    void process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const;


    // New interface

    void classify(const ed::Entity& e, const std::string& property, const ed::perception::CategoricalDistribution& prior,
                  ed::perception::ClassificationOutput& output) const {}

    void addTrainingInstance(const ed::Entity& e, const std::string& property, const std::string& value) {}

    void train() {}

    void loadRecognitionData(const std::string& path) {}

    void saveRecognitionData(const std::string& path) const {}

};

#endif

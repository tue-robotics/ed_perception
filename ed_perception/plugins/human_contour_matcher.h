/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: July 2015
*/

#ifndef ED_PERCEPTION_HUMAN_CONTOUR_MATCHER_H_
#define ED_PERCEPTION_HUMAN_CONTOUR_MATCHER_H_

#include <ed/perception/module.h>

#include "human_classifier.h"

class HumanContourMatcher : public ed::perception::Module
{

private:
    HumanClassifier human_classifier_;

    bool init_success_;

    std::string	module_name_;    /*!< Name of the module, for output */
    std::string module_path_;
    bool debug_mode_;
    std::string debug_folder_;

    std::string template_front_path_;
    std::string template_left_path_;
    std::string template_right_path_;

    bool face_detect_enabled_;
    int match_iterations_;
    int dt_line_width_;
    int border_size_;
    int num_slices_matching_;
    double max_template_err_;

    double type_positive_score_;
    double type_negative_score_;
    double type_unknown_score_;

public:

    HumanContourMatcher();

    virtual ~HumanContourMatcher();

    void configure(tue::Configuration config);

    void loadConfig(const std::string& config_path);

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

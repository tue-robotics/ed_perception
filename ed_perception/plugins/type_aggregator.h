/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: July 2015
*/

#ifndef ED_TYPE_AGGREGATOR_H_
#define ED_TYPE_AGGREGATOR_H_

#include <ed/perception/module.h>

// OpenCV includes
#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>


class TypeAggregator : public ed::perception::Module
{
    struct DictionaryMatch{
        uint matches;
        float score;
        std::string entry;
    };

    struct Feature{
        std::string name;
        std::string plugin_name;
        float score;

        Feature(std::string name_local, std::string plugin_name_local, float score_local){
            name = name_local;
            plugin_name = plugin_name_local;
            score = score_local;
        }

        bool operator== (const Feature &f)
        {
            return f.name == name;
        }
    };

/*
* ###########################################
*  				    PUBLIC
* ###########################################
*/

public:

    TypeAggregator();

    virtual ~TypeAggregator();

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

/*
* ###########################################
*  				PRIVATE
* ###########################################
*/

private:

    // module configuration
    bool init_success_;
    std::string	module_name_;    /*!< Name of the module, for output */

    std::vector<std::string> plugin_names_;
    float positive_tresh_;

    // Collect the features asserted to the entity configuration
    void collectFeatures(tue::Configuration entity_conf, std::vector<Feature>& features, std::vector<Feature>& hypothesis) const;

    // match the hypothesis collected, and return the most likely
    void matchHypothesis(std::vector<Feature>& features, std::map<std::string, float>& type_histogram, std::string& type, float& score) const;

    // filter the type according to some rules like size, if it has a face or human shape, etc...
    void determineType(std::vector<Feature>& features, std::string& type, float& score) const;

    // calculate a score
    float weightedScore(float score, std::string plugin_name) const;
};

#endif

#ifndef ED_TYPE_AGGREGATOR_H_
#define ED_TYPE_AGGREGATOR_H_

#include <ed/perception_modules/perception_module.h>

// OpenCV includes
#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

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


class TypeAggregator : public ed::PerceptionModule
{


/*
* ###########################################
*  				    PUBLIC
* ###########################################
*/

public:

    TypeAggregator();

    virtual ~TypeAggregator();

    void loadConfig(const std::string& config_path);

    void process(ed::EntityConstPtr e, tue::Configuration& result) const;


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

    void collectFeatures(tue::Configuration entity_conf, std::vector<Feature>& features, std::vector<Feature>& hypothesis) const;

    void matchHypothesis(std::vector<Feature>& features, std::map<std::string, float>& type_histogram, std::string& type, float& score) const;

    void determineType(std::vector<Feature>& features, std::string& type, float& score) const;

    bool loadDictionary(const std::string path);

    float weightedScore(float score, std::string plugin_name) const;
};

#endif

#ifndef ED_TYPE_AGGREGATOR_H_
#define ED_TYPE_AGGREGATOR_H_

#include <ed/perception_modules/perception_module.h>

// OpenCV includes
#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

struct dictionary_match{
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

typedef std::pair<std::string, float> StringFloatPair;
struct CompareValue
{
    bool operator()(const StringFloatPair& a, const StringFloatPair& b) const
    {
        return a.second < b.second;
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
    std::string	kModuleName;    /*!< Name of the module, for output */

    std::vector<std::string> kPluginNames;
    std::map<std::string, std::vector<std::string> > dictionary;

    float kPositiveTresh;

    void collect_features(tue::Configuration& entity_conf,
                          std::vector<Feature>& features,
                          std::vector<Feature>& hypothesis) const;

    void match_hypothesis(std::vector<Feature>& features,
                        std::map<std::string, float>& type_histogram,
                        std::string& type,
                        float& score) const;

    void discard_options(std::vector<Feature>& features,
                         std::string& type,
                         float& score) const;

    bool load_dictionary(const std::string path);

    float normalize(float x, float min, float max) const;
};

#endif

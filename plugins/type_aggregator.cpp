#include "type_aggregator.h"
#include "ed/measurement.h"
#include <ed/entity.h>
#include <algorithm>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <tue/config/reader.h>


// ----------------------------------------------------------------------------------------------------

TypeAggregator::TypeAggregator():
    PerceptionModule("type_aggregator"),
    init_success_(false)
{
}

// ----------------------------------------------------------------------------------------------------

TypeAggregator::~TypeAggregator()
{
}

// ----------------------------------------------------------------------------------------------------

void TypeAggregator::loadConfig(const std::string& config_path) {
    module_name_ = "type_aggregator";

    plugin_names_.push_back("human_contour_matcher");
    plugin_names_.push_back("face_detector");
    plugin_names_.push_back("size_matcher");
    plugin_names_.push_back("odu_finder");
    plugin_names_.push_back("color_matcher");

    positive_tresh_ = 0.5;

    init_success_ = true;
    std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void TypeAggregator::process(ed::EntityConstPtr e, tue::Configuration& entity_conf) const
{

    // if initialization failed, return
    if (!init_success_)
        return;

    std::vector<Feature> features;
    std::vector<Feature> hypothesis;
    std::map<std::string, float> type_histogram;
    std::string type = "";
    float best_score = 0;
    float min;

    tue::config::Reader old_entity_conf(e->data());

    // rebuild histogram from previous configuration
    if (old_entity_conf.readGroup("perception_result", tue::config::OPTIONAL))
    {
        if (old_entity_conf.readArray("histogram", tue::config::OPTIONAL))
        {
            while(old_entity_conf.nextArrayItem())
            {
                std::string type;
                float amount;

                if (old_entity_conf.value("type", type, tue::config::OPTIONAL) && old_entity_conf.value("amount", amount, tue::config::OPTIONAL))
                {
//                    type_histogram.insert(std::pair<std::string, float>(type, amount));
                }else{
                    std::cout << "[" << module_name_ << "] " << "Malformed histogram entry. type = " << type << ", amount = " << amount << std::endl;
                }
            }
            old_entity_conf.endArray();
        }
        old_entity_conf.endGroup();
    }


    // collect features asserted by other perception plugins
    collectFeatures(entity_conf, features, hypothesis);

    // match hypothesis from different plugins
    matchHypothesis(hypothesis, type_histogram, type, best_score);

    // discard hypothesis based on the features
    discardOptions(features, type, best_score);

    // assert general type
    if (!type.empty()){
        entity_conf.setValue("type", type);
    }

    // create or read perception_result group
    if (!entity_conf.readGroup("perception_result", tue::OPTIONAL))
    {
        entity_conf.writeGroup("perception_result");
    }

    // Update histogram in the configuration
    entity_conf.writeArray("histogram");
    for(std::map<std::string, float>::const_iterator it = type_histogram.begin(); it != type_histogram.end(); ++it)
    {
        entity_conf.addArrayItem();
        entity_conf.setValue("type", it->first);
        entity_conf.setValue("amount", it->second);
        entity_conf.endArrayItem();
    }
    entity_conf.endArray(); // close histogram array

    entity_conf.writeGroup("type_aggregator");

    // assert type_aggregator type
    if (!type.empty()){
        entity_conf.setValue("type", type);
        entity_conf.setValue("score", best_score);

//        std::cout << "[" << kModuleName << "] " << "Asserted type: " << type << " (" << certainty << ")" << std::endl;
    }else{
//        std::cout << "[" << kModuleName << "] " << "No hypothesis found." << std::endl;
    }

    entity_conf.endGroup(); // close type_aggregator group
    entity_conf.endGroup(); // close perception_result group
}

// ----------------------------------------------------------------------------------------------------


void TypeAggregator::discardOptions(std::vector<Feature>& features, std::string& type, float& score) const{

    bool face = false;
    bool human_shape = false;
    bool small_size = false;
    bool medium_size = false;
    bool large_size = false;


    // get booleans from features
    for (std::vector<Feature>::iterator feat_it = features.begin() ; feat_it != features.end(); ++feat_it){
        face = ((feat_it->name.compare("face") == 0 && feat_it->score == 1) || face == true);
        human_shape = ((feat_it->name.compare("human_shape") == 0 && feat_it->score == 1) || human_shape == true);
        small_size = ((feat_it->name.compare("small_size") == 0 && feat_it->score == 1) || small_size == true);
        medium_size = ((feat_it->name.compare("medium_size") == 0 && feat_it->score == 1) || medium_size == true);
        large_size = ((feat_it->name.compare("large_size") == 0 && feat_it->score == 1) || large_size == true);
    }

    // assuming that hypothesis are only used for household objects, therefore small
    //  anything medium or big cannot have an hypothesis

    if ((face || (human_shape && face)) && !small_size){
        type = "human";
        score = 1;
    }else if(!human_shape && !face && !small_size){
        type = "";
        score = 0;
    }
}

// ----------------------------------------------------------------------------------------------------


void TypeAggregator::matchHypothesis(std::vector<Feature>& features,
                                    std::map<std::string, float>& type_histogram,
                                    std::string& type,
                                    float& score) const{

    std::vector<Feature>::iterator feat_it;
    std::map<std::string, float>::iterator hist_it;

    float min;

    for (feat_it = features.begin() ; feat_it != features.end(); ++feat_it){
        // search for match with the dictionary
        hist_it = type_histogram.find(feat_it->name);

        // add a new entry or update the existing one
        if (hist_it != type_histogram.end()){
//            std::cout << "[" << kModuleName << "] " << "Update entry: " << feat_it->name << ", " << hist_it->second  << " + " << feat_it->score << std::endl;
            hist_it->second += weightedScore(feat_it->score, feat_it->plugin_name);
//            hist_it->second = hist_it->second*0.33 + feat_it->score*0.66;
        }else{
//            std::cout << "[" << kModuleName << "] " << "New entry: " << feat_it->name << ", " << feat_it->score << std::endl;
            type_histogram.insert(std::pair<std::string, float>(feat_it->name, weightedScore(feat_it->score, feat_it->plugin_name)));
//            type_histogram.insert(std::pair<std::string, float>(feat_it->name, feat_it->score*0.66));
        }
    }

    min = std::numeric_limits<float>::max();
    score = 0;
    for(hist_it = type_histogram.begin(); hist_it != type_histogram.end(); ++hist_it) {
        // save min
        if (min > hist_it->second) min = hist_it->second;

        // save max
        if (score < hist_it->second){
            type = hist_it->first;
            score = hist_it->second;
        }
    }
}


// ----------------------------------------------------------------------------------------------------

float TypeAggregator::weightedScore(float score, std::string plugin_name) const{

   return score; // for now
}


// ----------------------------------------------------------------------------------------------------

void TypeAggregator::collectFeatures(tue::Configuration& entity_conf, std::vector<Feature>& features, std::vector<Feature>& hypothesis) const{

    float score = 0;
    std::string feat_name = "";

    // find perception_result group
    if (entity_conf.readGroup("perception_result"))
    {
        // find perception plugins that already processed the entity
        for(std::vector<std::string>::const_iterator pluginName = plugin_names_.begin(); pluginName != plugin_names_.end(); ++pluginName) {
            // visit every perception plugin
            if (entity_conf.readGroup(*pluginName)){

                // collect Features
                if (entity_conf.value("score", score, tue::OPTIONAL) && entity_conf.value("label", feat_name, tue::OPTIONAL)){
                    features.push_back(Feature(feat_name, *pluginName, score));
                }

                // collect Hypothesis
                if (entity_conf.readArray("hypothesis", tue::OPTIONAL)){
                    // iterate through the hypothesis
                    while(entity_conf.nextArrayItem())
                    {
                        // if the hypothesis has a name and score
                        if (entity_conf.value("name", feat_name, tue::OPTIONAL) && entity_conf.value("score", score, tue::OPTIONAL))
                        {
                            hypothesis.push_back(Feature(feat_name, *pluginName, score));
//                            std::cout << "Feature " << feat_name << ", from " << *pluginName << " with " << score << std::endl;
                        }
                    }
                    entity_conf.endArray();
                }

                // close the group just read
                entity_conf.endGroup();
            }else{
//                std::cout << "Didnt find group " << *pluginName << std::endl;
            }
        }

        // close perception_result group
        entity_conf.endGroup();
    }
}

// ----------------------------------------------------------------------------------------------------

float TypeAggregator::normalize(float x, float min, float max) const{
    return (x - min) / max;
}

ED_REGISTER_PERCEPTION_MODULE(TypeAggregator)

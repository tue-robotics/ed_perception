#include "type_aggregator.h"
#include "ed/measurement.h"
#include <ed/entity.h>
#include <algorithm>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <tue/config/reader.h>


// ----------------------------------------------------------------------------------------------------

TypeAggregator::TypeAggregator():
    PerceptionModule("type_aggregator")
{
}

// ----------------------------------------------------------------------------------------------------

TypeAggregator::~TypeAggregator()
{
}

// ----------------------------------------------------------------------------------------------------

void TypeAggregator::loadConfig(const std::string& config_path) {
    kModuleName = "type_aggregator";

    kPluginNames.push_back("human_contour_matcher");
    kPluginNames.push_back("face_detector");
    kPluginNames.push_back("size_matcher");
    kPluginNames.push_back("odu_finder");
    kPluginNames.push_back("color_matcher");

    kPositiveTresh = 0.5;

//    std::cout << "[" << kModuleName << "] " << "Loading dictionary ..." << config_path + "/type_dictionary.yml" << std::endl;

//    if (load_dictionary(config_path + "/type_dictionary.yml"))
//        std::cout << "[" << kModuleName << "] " << "Ready!" << std::endl;
//    else
//        std::cout << "[" << kModuleName << "] " << "Unable to load dictionary from " << config_path + "/type_dictionary.yml" << std::endl;
//        return;

    init_success_ = true;
}

// ----------------------------------------------------------------------------------------------------

void TypeAggregator::process(ed::EntityConstPtr e, tue::Configuration& entity_conf) const
{

    // if initialization failed, return
//    if (!init_success_)
//        return;

    std::map<std::string, std::map<std::string, float> > hypothesis;   // [label | [plugin who named it | score]]
    std::vector<Feature> features;    // [hypothesis name | [plugin who named it | score]]
    std::map<std::string, float> type_histogram;
    std::string type = "";
    float best_score = 0;
    float min;
    float max;

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
                    std::cout << "[" << kModuleName << "] " << "Malformed histogram entry." << type << ", " << amount << std::endl;
                }
            }
            old_entity_conf.endArray();
        }
        old_entity_conf.endGroup();
    }


    // collect features asserted by other perception plugins
    collect_features(entity_conf, features);

    // match these features against the features dictionary
    match_features(features, type_histogram, type, best_score, min, max);

    // assert type
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
//        entity_conf.setValue("amount", normalize(it->second, min, max));
        entity_conf.endArrayItem();
    }
    entity_conf.endArray(); // close histogram array

    entity_conf.writeGroup("type_aggregator");
    // assert type
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


void TypeAggregator::match_features(std::vector<Feature>& features,
                                    std::map<std::string, float>& type_histogram,
                                    std::string& type,
                                    float& best_score, float min, float max) const{

    std::map<std::string, std::pair<std::string, float> >::const_iterator feat_it;
    std::map<std::string, std::vector<std::string> >::const_iterator dict_it;
    std::map<std::string, float>::iterator hist_it;


    for (std::vector<Feature>::iterator feat_it = features.begin() ; feat_it != features.end(); ++feat_it){
        // search for match with the dictionary
        hist_it = type_histogram.find(feat_it->name);

        // add a new entry or update the existing one
        if (hist_it != type_histogram.end()){
//            std::cout << "[" << kModuleName << "] " << "Update entry: " << feat_it->name << ", " <<
//                         hist_it->second  << " + " << feat_it->score << std::endl;
            hist_it->second += feat_it->score;
        }else{
//            std::cout << "[" << kModuleName << "] " << "New entry: " << feat_it->name << ", " << feat_it->score << std::endl;
            type_histogram.insert(std::pair<std::string, float>(feat_it->name, feat_it->score));
        }
    }

    // find best, min and max result in the hystogram
    max = 0;
    min = std::numeric_limits<float>::max();
    for(hist_it = type_histogram.begin(); hist_it != type_histogram.end(); ++hist_it) {
        // remember best
        if (best_score < hist_it->second){
            best_score = hist_it->second;
            type = hist_it->first;
        }
        // remember min
        if (min > hist_it->second) min = hist_it->second;

        // remember max
        if (max < hist_it->second) max = hist_it->second;
    }
}


// ----------------------------------------------------------------------------------------------------

void TypeAggregator::collect_features(tue::Configuration& entity_conf, std::vector<Feature>& features) const{

    float score = 0;
    std::string feat_name = "";

    // find perception_result group
    if (entity_conf.readGroup("perception_result"))
    {
        // find perception plugins that already processed the entity
        for(std::vector<std::string>::const_iterator pluginName = kPluginNames.begin(); pluginName != kPluginNames.end(); ++pluginName) {
            // visit every perception plugin
            if (entity_conf.readGroup(*pluginName)){

                // collect Features
                if (entity_conf.value("score", score, tue::OPTIONAL) && entity_conf.value("label", feat_name, tue::OPTIONAL))
                {
                    std::pair<std::string, float> inner_pair (*pluginName, score);
//                    features.insert(std::pair<std::string, std::pair<std::string, float> >(feat_name, inner_pair));
//                    std::cout << "Feature " << feat_name << ", from " << *pluginName << " with " << score << std::endl;
                }

                // collect Hypothesis
                if (entity_conf.readArray("hypothesis", tue::OPTIONAL)){
                    // iterate through the hypothesis
                    while(entity_conf.nextArrayItem())
                    {
                        // if the hypothesis has a name and score
                        if (entity_conf.value("name", feat_name, tue::OPTIONAL) && entity_conf.value("score", score, tue::OPTIONAL))
                        {
                            features.push_back(Feature(feat_name, *pluginName, score));
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

bool TypeAggregator::load_dictionary(const std::string path) {
    if (path.empty()){
        std::cout << "[" << kModuleName << "] " << "Dictionary path not specified." << std::endl;
        return false;
    }else{
        tue::Configuration dictionary_conf;
        std::string type_entry;
        std::string feature_name;
        std::vector<std::string> features;

        // load list of models to include
        if (dictionary_conf.loadFromYAMLFile(path)){

            if (dictionary_conf.readArray("conclusions")){
                while(dictionary_conf.nextArrayItem())
                {
                    if (dictionary_conf.value("type", type_entry))
                    {
                        features.clear();
                        if (dictionary_conf.readArray("features"))
                        {
                            while(dictionary_conf.nextArrayItem())
                            {
                                if (dictionary_conf.value("name", feature_name))
                                {
                                    features.push_back(feature_name);
                                }
                            }
                        }
                        dictionary_conf.endArray();
                        dictionary.insert(std::pair<std::string, std::vector<std::string> >(type_entry, features));
                    }
                }
                dictionary_conf.endArray();
            }else{
                std::cout << "[" << kModuleName << "] " << "Dictionary incorrectly built." << std::endl;
                std::cout << dictionary_conf.error() << std::endl;
                return false;
            }
        }else{
            std::cout << "[" << kModuleName << "] " << "Dictionary not found at " << path << "." << std::endl;
            std::cout << dictionary_conf.error() << std::endl;
            return false;
        }
    }
    return true;
}


// ----------------------------------------------------------------------------------------------------

float TypeAggregator::normalize(float x, float min, float max) const{
    return (x - min) * max;
}

ED_REGISTER_PERCEPTION_MODULE(TypeAggregator)

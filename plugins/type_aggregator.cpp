/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#include "type_aggregator.h"
#include "ed/measurement.h"
#include <ed/entity.h>
#include <ed/error_context.h>

#include <algorithm>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <tue/config/reader.h>


// ----------------------------------------------------------------------------------------------------

TypeAggregator::TypeAggregator():
    ed::perception::Module("type_aggregator"),
    init_success_(false)
{
}

// ----------------------------------------------------------------------------------------------------

TypeAggregator::~TypeAggregator()
{
}

// ----------------------------------------------------------------------------------------------------


void TypeAggregator::configure(tue::Configuration config) {

    if (!config.value("classification_threshold", positive_tresh_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'classification_threshold' not found. Using default: " << positive_tresh_ << std::endl;

}


// ----------------------------------------------------------------------------------------------------

void TypeAggregator::loadConfig(const std::string& config_path) {
    module_name_ = "type_aggregator";

    plugin_names_.push_back("human_contour_matcher");
    plugin_names_.push_back("face_detector");
    plugin_names_.push_back("size_matcher");
    plugin_names_.push_back("odu_finder");
    plugin_names_.push_back("color_matcher");

    // default values in case configure(...) is not called!
    positive_tresh_ = 0.5;

    init_success_ = true;
    std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;

}

// ----------------------------------------------------------------------------------------------------

void TypeAggregator::process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const
{
    ed::ErrorContext errc("Processing entity in TypeAggregator");

    const ed::EntityConstPtr& e = input.entity;
    tue::Configuration& entity_conf = output.data;

    // if initialization failed, return
    if (!init_success_)
        return;

/*
    // ---------- Prepare measurement ----------

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->lastMeasurement();
    if (!msr)
        return;

    uint min_x, max_x, min_y, max_y;

    // create a view
    rgbd::View view(*msr->image(), msr->image()->getRGBImage().cols);

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // crop it to match the view
    cv::Mat cropped_image(color_image(cv::Rect(0,0,view.getWidth(), view.getHeight())));

    // initialize bounding box points
    max_x = 0;
    max_y = 0;
    min_x = view.getWidth();
    min_y = view.getHeight();

    cv::Mat mask = cv::Mat::zeros(view.getHeight(), view.getWidth(), CV_8UC1);
    // Iterate over all points in the mask
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(view.getWidth()); it != msr->imageMask().end(); ++it)
    {
        // mask's (x, y) coordinate in the depth image
        const cv::Point2i& p_2d = *it;

        // paint a mask
        mask.at<unsigned char>(*it) = 255;

        // update the boundary coordinates
        if (min_x > p_2d.x) min_x = p_2d.x;
        if (max_x < p_2d.x) max_x = p_2d.x;
        if (min_y > p_2d.y) min_y = p_2d.y;
        if (max_y < p_2d.y) max_y = p_2d.y;
    }

    cv::imwrite((std::string)"/tmp/type_aggregator/" + e->id().c_str() + ".png", cropped_image(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y)));
*/
    // ---------------------------------------------

    std::vector<Feature> features;
    std::vector<Feature> hypothesis;
    std::map<std::string, float> type_histogram;
    std::string type = "";
    float best_score = 0;

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
                    std::cout << "[" << module_name_ << "] " << "Malformed histogram! (type = " << type << ", amount = " << amount <<")" << std::endl;
                }
            }
            old_entity_conf.endArray();
        }
        old_entity_conf.endGroup();
    }


    // collect features asserted by other perception plugins
    collectFeatures(entity_conf.limitScope(), features, hypothesis);

    // match hypothesis from different plugins
    matchHypothesis(hypothesis, type_histogram, type, best_score);

    // discard hypothesis based on the features
    determineType(features, type, best_score);

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

//    std::cout << "[" << module_name_ << "] " << "######### Entity: " << e->id() << "#########" << std::endl;
//    std::cout << entity_conf << std::endl;
//    std::cout << std::endl;

}


// ----------------------------------------------------------------------------------------------------


void TypeAggregator::determineType(std::vector<Feature>& features, std::string& type, float& score) const{

    bool face = false;
    bool multiple_faces = false;
    bool human_shape = false;
    bool small_size = false;
    bool medium_size = false;
    bool large_size = false;


    // get booleans from features
    for (std::vector<Feature>::iterator feat_it = features.begin() ; feat_it != features.end(); ++feat_it){
        face = ((feat_it->name.compare("face") == 0 && feat_it->score == 1) || face == true);
        multiple_faces = ((feat_it->name.compare("multiple_faces") == 0 && feat_it->score == 1) || multiple_faces == true);
        human_shape = ((feat_it->name.compare("human_shape") == 0 && feat_it->score == 1) || human_shape == true);

        small_size = ((feat_it->name.compare("small_size") == 0 && feat_it->score == 1) || small_size == true);
        medium_size = ((feat_it->name.compare("medium_size") == 0 && feat_it->score == 1) || medium_size == true);
        large_size = ((feat_it->name.compare("large_size") == 0 && feat_it->score == 1) || large_size == true);
    }

    // if it has a face or human shape, and its not small, it must be a human
    if ((face || (human_shape && !multiple_faces)) && (medium_size || large_size)){
        type = "human";
        score = 1;
    }else if (multiple_faces && (medium_size || large_size)){
        type = "crowd";
        score = 1;
    // if it has no face, no human shape, and its not small, then its not an ordinary object
    }else if(!human_shape && !face && !multiple_faces && (medium_size || large_size)){
        type = "unknown";
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


void TypeAggregator::collectFeatures(tue::Configuration entity_conf, std::vector<Feature>& features, std::vector<Feature>& hypothesis) const{

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


ED_REGISTER_PERCEPTION_MODULE(TypeAggregator)

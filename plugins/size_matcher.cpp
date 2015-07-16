/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#include "size_matcher.h"

#include "ed/measurement.h"
#include "ed/entity.h"
#include <ed/error_context.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

// Loading models
#include <fstream>


// NOTE: currently only looks at hight!

// ----------------------------------------------------------------------------------------------------

SizeMatcher::SizeMatcher() :
    Module("size_matcher"),
    init_success_(false)
{}

// ----------------------------------------------------------------------------------------------------

SizeMatcher::~SizeMatcher(){}

// ----------------------------------------------------------------------------------------------------


void SizeMatcher::configure(tue::Configuration config) {

    if (!config.value("size_diff_threshold", size_diff_threshold_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'size_diff_threshold' not found. Using default: " << size_diff_threshold_ << std::endl;

    if (!config.value("small_size_treshold", small_size_treshold_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'small_size_treshold' not found. Using default: " << small_size_treshold_ << std::endl;

    if (!config.value("medium_size_treshold", medium_size_treshold_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'medium_size_treshold' not found. Using default: " << medium_size_treshold_ << std::endl;

    if (!config.value("type_positive_score", type_positive_score_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'type_positive_score' not found. Using default: " << type_positive_score_ << std::endl;

    if (!config.value("type_negative_score", type_negative_score_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'type_negative_score' not found. Using default: " << type_negative_score_ << std::endl;

    if (!config.value("type_unknown_score", type_unknown_score_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'type_unknown_score' not found. Using default: " << type_unknown_score_ << std::endl;


    if (models_.empty())
         std::cout << "[" << module_name_ << "] " << "No information from the models size was loaded!" << std::endl;
    else
        std::cout << "[" << module_name_ << "] " << "Loaded size information for " << models_.size() << " different models" << std::endl;

    init_success_ = true;

    std::cout << "[" << module_name_ << "] " << "Ready!"<< std::endl;
}

// ----------------------------------------------------------------------------------------------------

void SizeMatcher::loadConfig(const std::string& config_path) {

    module_name_ = "size_matcher";

    // default values in case configure(...) is not called!
    size_diff_threshold_ = 0.8;
    small_size_treshold_ = 0.5;
    medium_size_treshold_ = 0.7;
    type_positive_score_ = 0.9;
    type_negative_score_ = 0.4;
    type_unknown_score_ = 0.05;
}

// ----------------------------------------------------------------------------------------------------

void SizeMatcher::loadModel(const std::string& model_name, const std::string& model_path)
{

    std::string models_folder = model_path.substr(0, model_path.find_last_of("/") - 1); // remove last slash
    models_folder = models_folder.substr(0, models_folder.find_last_of("/"));   // remove size_matcher from path

    std::string path = models_folder + "/models/" + model_name +  "/" +  model_name + ".yml";

    if (loadLearnedModel(path, model_name)){
//        std::cout << "[" << module_name_ << "] " << "Loaded sizes for " << model_name << std::endl;
    }
    else{
//        std::cout << "[" << module_name_ << "] " << "Couldn not load sizes for " << path << "!" << std::endl;
    }
}


// ----------------------------------------------------------------------------------------------------

void SizeMatcher::process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const
{
    const ed::EntityConstPtr& e = input.entity;
    tue::Configuration& result = output.data;

    ed::ErrorContext errc("Processing entity in SizeMatcher");

    if (!init_success_)
        return;

    const ed::ConvexHull& conv_hull = e->convexHull();

    // if there is no convex hull available, cancel classification
    if (conv_hull.points.empty())
        return;

    std::map<std::string, double> hypothesis;

    // initialize size class
    bool size_small = false;
    bool size_medium = false;
    bool size_big = false;

    double object_height = conv_hull.height();
    double object_area = conv_hull.area;

    if (object_area <= 0)
        std::cout << "[" << module_name_ << "] " << "Bad area value for the entity (area = " << object_area << ")" << std::endl;

    // set object size class
    if (object_area < small_size_treshold_){
        size_small = true;
    }else if (small_size_treshold_ < object_area && object_area < medium_size_treshold_){
        size_medium = true;
    }else if (medium_size_treshold_ < object_area){
        size_big = true;
    }else
        std::cout << "[" << module_name_ << "] " << "Could not set a size threshold!" << std::endl;

    // compare object size to loaded models
    for(std::map<std::string, std::vector<ObjectSize> >::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const std::string& label = it->first;
        const std::vector<ObjectSize>& sizes = it->second;

        double best_score = 0;
        for(std::vector<ObjectSize>::const_iterator it_size = sizes.begin(); it_size != sizes.end(); ++it_size)
        {
            const ObjectSize& model_size = *it_size;

            double h_ratio = object_height / model_size.height;
            double a_ratio = object_area / model_size.area;

            double h_score;
            if (h_ratio > 1.5)
                h_score = 0;
            else
            {
                if (h_ratio > 1)
                    h_ratio = 1 / h_ratio;

                h_score = h_ratio;
            }

            double a_score;
            if (a_ratio > 1.5)
                a_score = 0;
            else
            {
                if (a_ratio > 1)
                    a_ratio = 1 / a_ratio;

                a_score = a_ratio;
            }

            double final_score;

            // temporary bug fix when area = 0
            if (a_score == 0)
                final_score = h_score;
            else
                final_score = h_score * a_score;

            // update best score
            best_score = std::max(best_score, final_score);
        }

        hypothesis[label] = 0.5 * best_score; // TODO: magic number
    }

    // ----------------------- ASSERT RESULTS -----------------------

    // create group if it doesnt exist
    if (!result.readGroup("perception_result", tue::OPTIONAL))
    {
        result.writeGroup("perception_result");
    }

    result.writeGroup("size_matcher");

    result.writeGroup("size");
    result.setValue("height", object_height);
    result.setValue("area", object_area);
    result.endGroup();

    // assert hypothesis
    if (!hypothesis.empty()){
        for (std::map<std::string, double>::const_iterator it = hypothesis.begin(); it != hypothesis.end(); ++it){
//                output.type_update.setScore(it->first, std::max(it->second/2, type_negative_score_));

            // if the best score is above 0 assert that, otherwise assert a negative score
            if (it->second > 0)
                output.type_update.setScore(it->first, it->second);
            else
                output.type_update.setScore(it->first, type_negative_score_);

            // std::cout << "[" << module_name_ << "] " << "Asserting: " << it->first << ": " << it->second << std::endl;
        }
    }

    // Set labels and scores
    if (size_small)
    {
        result.setValue("label", "small_size");

        // if its small, for sure its not a person
        output.type_update.setScore("human", type_negative_score_);
//        output.type_update.setScore("crowd", type_negative_score_);

    }else if (size_medium){
        result.setValue("label", "medium_size");

//        output.type_update.setScore("human", type_positive_score_);
//        output.type_update.setScore("crowd", type_positive_score_);

    }else if (size_big){
        result.setValue("label", "large_size");

        // output.type_update.setScore("human", type_positive_score_);
//        output.type_update.setScore("crowd", 0.5);

//        output.type_update.setScore("human", type_positive_score_);
//        output.type_update.setScore("crowd", type_positive_score_);
    }else
        std::cout << "[" << module_name_ << "] " << "Could not get size class!" << std::endl;

    output.type_update.setUnknownScore(type_unknown_score_);

    result.endGroup();  // close size_matcher group
    result.endGroup();  // close perception_result group
}


// ----------------------------------------------------------------------------------------------------


bool SizeMatcher::loadLearnedModel(std::string path, std::string model_name){
    if (path.empty())
    {
        std::cout << "[" << module_name_ << "] " << "Empty path!" << path << std::endl;
        return false;
    }

    tue::Configuration conf;
    double height;
    double area;
    std::vector<ObjectSize> model_sizes;

    if (!conf.loadFromYAMLFile(path))
    {
//        std::cout << "Could not load " << path << std::endl;
        return false;
    }

    if (!conf.readGroup("model"))
    {
        std::cout << "[" << module_name_ << "] " << "Could not find 'model' group" << std::endl;
        return false;
    }

    if (conf.readArray("size")) // read Size arary
    {
        while(conf.nextArrayItem())
        {
            if (conf.value("height", height, tue::OPTIONAL) && conf.value("area", area, tue::OPTIONAL))  // read height and area
            {
                ObjectSize obj_sz(0, height, area);
                model_sizes.push_back(obj_sz);
            }
            else
                std::cout << "[" << module_name_ << "] " << "Could not find 'height' and 'area' values" << std::endl;
        }

        if (!model_sizes.empty())  // save sizes to map
            models_[model_name] = model_sizes;
        else
            std::cout << "[" << module_name_ << "] " << "Could not read any sizes" << std::endl;

        conf.endArray();    // close Size array
    }else
        std::cout << "[" << module_name_ << "] " << "Could not find 'size' group" << std::endl;

    conf.endGroup();  // close Model group

    return true;
}

ED_REGISTER_PERCEPTION_MODULE(SizeMatcher)


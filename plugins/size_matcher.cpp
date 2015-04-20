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

    if (!config.value("small_size_treshold", size_diff_threshold_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'small_size_treshold' not found. Using default: " << small_size_treshold_ << std::endl;

    if (!config.value("medium_size_treshold", medium_size_treshold_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'medium_size_treshold' not found. Using default: " << medium_size_treshold_ << std::endl;

    if (!config.value("type_positive_score", type_positive_score_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'type_positive_score' not found. Using default: " << type_positive_score_ << std::endl;

    if (!config.value("type_negative_score", type_negative_score_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'type_negative_score' not found. Using default: " << type_negative_score_ << std::endl;

    if (!config.value("type_unknown_score", type_unknown_score_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'type_unknown_score' not found. Using default: " << type_unknown_score_ << std::endl;

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

    const ed::ConvexHull2D& chull = e->convexHull();

    // if there is no convex hull available, cancel classification
    if (chull.chull.empty())
        return;

    std::map<std::string, double> hypothesis;

    // initialize size class
    bool size_small = false;
    bool size_medium = false;
    bool size_big = false;

    double object_height = chull.height();
    double object_width = 0;
    double object_area = chull.area();


    // set object size class
    if ((object_width + object_height) < small_size_treshold_){
        size_small = true;
    }else if (small_size_treshold_ < (object_width + object_height) && (object_width + object_height) < medium_size_treshold_){
        size_medium = true;
    }else if (medium_size_treshold_ < (object_width + object_height)){
        size_big = true;
    }else
        std::cout << "[" << module_name_ << "] " << "Could not set a size threshold!" << std::endl;


//std::cout << "Entity size: " << object_height << " - " << object_area << std::endl;

    // compare object size to loaded models
    for(std::map<std::string, std::vector<ObjectSize> >::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const std::string& label = it->first;
        const std::vector<ObjectSize>& sizes = it->second;

        double best_score = 0;

        for(std::vector<ObjectSize>::const_iterator it_size = sizes.begin(); it_size != sizes.end(); ++it_size)
        {
            double score;
            const ObjectSize& model_size = *it_size;

            // width and height difference
            double diff_w = std::abs(model_size.width - object_width);
            double diff_h = std::abs(model_size.height - object_height);


            if (diff_w > size_diff_threshold_ || diff_h > size_diff_threshold_)
                score = 0;
            else
            {
                double w_score = 1.0 - (diff_w / size_diff_threshold_); // TODO: magic score function
                double h_score = 1.0 - (diff_h / size_diff_threshold_); // TODO: magic score function
                score = (w_score + h_score) / 2;
            }

//std::cout << "Diffs: " << diff_w << ", " << diff_h << " \t Score: " << score << std::endl;

            if (score > best_score)
                object_width = model_size.width;

            best_score = std::max(best_score, score);
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
    result.setValue("width", object_width);
    result.setValue("height", object_height);
    result.endGroup();

    // Set labels and scores
    if (size_small){
        result.setValue("label", "small_size");

        // if its small, for sure its not a person
        output.type_update.setScore("human", 0);
        output.type_update.setScore("crowd", 0);

        // assert hypothesis
        if (!hypothesis.empty()){
            for (std::map<std::string, double>::const_iterator it = hypothesis.begin(); it != hypothesis.end(); ++it)
            {
    //            result.addArrayItem();
    //            result.setValue("name", it->first);
    //            result.setValue("score", std::max(it->second, 0.0));
    //            result.endArrayItem();

                output.type_update.setScore(it->first, std::max(it->second, type_negative_score_));
            }
        }

    }else if (size_medium){
        result.setValue("label", "medium_size");

        output.type_update.setScore("human", type_positive_score_);
        output.type_update.setScore("crowd", type_positive_score_);

    }else if (size_big){
        result.setValue("label", "large_size");

        output.type_update.setScore("human", type_positive_score_);
        output.type_update.setScore("crowd", type_positive_score_);
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
    float width;
    float height;
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
            if (conf.value("height", height, tue::OPTIONAL) && conf.value("width", width, tue::OPTIONAL))  // read height and width
            {
                ObjectSize obj_sz(width, height);
                model_sizes.push_back(obj_sz);
            }
            else
                std::cout << "[" << module_name_ << "] " << "Could not find 'height' and 'width' values" << std::endl;
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


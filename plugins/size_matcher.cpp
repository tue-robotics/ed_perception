#include "size_matcher.h"

#include "ed/measurement.h"
#include "ed/entity.h"
#include <rgbd/Image.h>
#include <rgbd/View.h>

// Loading models
#include <fstream>

namespace ed
{
namespace perception
{

// ----------------------------------------------------------------------------------------------------

SizeMatcher::SizeMatcher() : PerceptionModule("size_matcher")
{
}

// ----------------------------------------------------------------------------------------------------

SizeMatcher::~SizeMatcher()
{
}

// ----------------------------------------------------------------------------------------------------

void SizeMatcher::loadConfig(const std::string& config_path) {

    module_name_ = "size_matcher";

    difference_thresh_ = 0.8;
    small_tresh_ = 0.5;
    medium_tresh_ = 0.7;
}

// ----------------------------------------------------------------------------------------------------

void SizeMatcher::loadModel(const std::string& model_name, const std::string& model_path)
{

    std::string models_folder = model_path.substr(0, model_path.find_last_of("/") - 1); // remove last slash
    models_folder = models_folder.substr(0, models_folder.find_last_of("/"));   // remove size_matcher from path

    std::string path = models_folder + "/models/" + model_name +  "/" +  model_name + ".yml";

    if (loadLearning(path, model_name))
        std::cout << "[" << module_name_ << "] " << "Loaded sizes for " << model_name << std::endl;
    else{
//        std::cout << "[" << module_name_ << "] " << "Couldn not load sizes for " << path << "!" << std::endl;
    }
}


// ----------------------------------------------------------------------------------------------------

void SizeMatcher::process(ed::EntityConstPtr e, tue::Configuration& result) const
{
    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->lastMeasurement();
    if (!msr)
        return;

    const cv::Mat& rgb_image = msr->image()->getRGBImage();

    std::map<std::string, double> hypothesis;
    double best_err;

    // initialize min and max coordinates
    geo::Vector3 min(1e10, 1e10, 1e10);
    geo::Vector3 max(-1e10, -1e10, -1e10);

    rgbd::View view(*msr->image(), msr->imageMask().width());

    // get minimun and maximum X, Y and Z coordinates of the object
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(view.getWidth()); it != msr->imageMask().end(); ++it)
    {
        const cv::Point2i& p_2d = *it;

        geo::Vector3 p;
        if (view.getPoint3D(p_2d.x, p_2d.y, p))
        {
//            geo::Vector3 p_MAP = msr->sensorPose() * p;
            geo::Vector3 p_MAP = p;     // use this only when sensorPose is not available

            min.x = std::min(min.x, p_MAP.x); max.x = std::max(max.x, p_MAP.x);
            min.y = std::min(min.y, p_MAP.y); max.y = std::max(max.y, p_MAP.y);
            min.z = std::min(min.z, p_MAP.z); max.z = std::max(max.z, p_MAP.z);
        }
    }

    // determine size
    geo::Vector3 size = max - min;
    double object_width = sqrt(size.x * size.x + size.z * size.z);
    double object_height = size.y;

    // compare object size to loaded models
    for(std::map<std::string, std::vector<ObjectSize> >::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const std::string& label = it->first;
        const std::vector<ObjectSize>& sizes = it->second;

        best_err = std::numeric_limits<double>::max();

        for(std::vector<ObjectSize>::const_iterator it_size = sizes.begin(); it_size != sizes.end(); ++it_size)
        {
            const ObjectSize& model_size = *it_size;

            double diff_h;
            double diff_w;

            // difference in height and width, between 0 and 1
            diff_h = std::abs(model_size.max_height - object_height) / std::max(model_size.max_height, object_height);
            diff_w = std::abs(model_size.max_width - object_width) / std::max(model_size.max_width, object_width);

            // error will be the highest average diff
            if (best_err > (diff_h + diff_w) / 2) best_err = (diff_h + diff_w) / 2;
        }

        if (best_err < difference_thresh_)
            hypothesis[label] = std::max(1 - best_err, 0.0);
    }

    // ----------------------- SAVE RESULTS -----------------------

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

    if ((object_width + object_height) < small_tresh_){
        result.setValue("label", "small_size");
    }else if (small_tresh_ < (object_width + object_height) && (object_width + object_height) < medium_tresh_){
        result.setValue("label", "medium_size");
    }else if (medium_tresh_ < (object_width + object_height)){
        result.setValue("label", "large_size");
    }

    result.setValue("score", 1.0);

    // assert hypothesis
    if (!hypothesis.empty()){
        result.writeArray("hypothesis");
        for (std::map<std::string, double>::const_iterator it = hypothesis.begin(); it != hypothesis.end(); ++it)
        {
            result.addArrayItem();
            result.setValue("name", it->first);
            result.setValue("score", std::max(it->second, 0.0));
            result.endArrayItem();
        }
        result.endArray();
    }

    result.endGroup();  // close size_matcher group
    result.endGroup();  // close perception_result group
}

// ----------------------------------------------------------------------------------------------------

bool SizeMatcher::loadLearning(std::string path, std::string model_name){
    if (path.empty()){
        std::cout << "[" << module_name_ << "] " << "Empty path!" << path << std::endl;
        return false;
    }else{
        tue::Configuration conf;
        float width;
        float height;
        std::vector<ObjectSize> model_sizes;

        if (conf.loadFromYAMLFile(path)){       // read YAML configuration
            if (conf.readGroup("model")){       // read Model group
                if (conf.readArray("size")){    // read Size arary
                    while(conf.nextArrayItem()){
                        if (conf.value("height", height, tue::OPTIONAL) && conf.value("width", width, tue::OPTIONAL)){  // read height and width
                            ObjectSize obj_sz(0, height, 0, width);
                            model_sizes.push_back(obj_sz);
                        }else
                            std::cout << "[" << module_name_ << "] " << "Could not find 'height' and 'width' values" << std::endl;
                    }

                    if (!model_sizes.empty()){  // save sizes to map
                        models_[model_name] = model_sizes;
                    }else
                        std::cout << "[" << module_name_ << "] " << "Could not read any sizes" << std::endl;

                    conf.endArray();    // close Size array
                }else
                    std::cout << "[" << module_name_ << "] " << "Could not find 'size' group" << std::endl;

              conf.endGroup();  // close Model group
            }else
                std::cout << "[" << module_name_ << "] " << "Could not find 'model' group" << std::endl;
        }else{
//            std::cout << "[" << module_name_ << "] " << "Didn't find configuration file." << std::endl;
            return false;
        }
    }
    return true;
}


// ----------------------------------------------------------------------------------------------------

bool SizeMatcher::loadSize(std::string path, std::string model_name){
    std::ifstream model_file;

    // Open file
    model_file.open(path.c_str());

    if (!model_file.is_open())
        return false;

    std::vector<ObjectSize> model_vec;

    // Get data from file
    double h_min = 0, h_max = 0, w_min = 0, w_max = 0;
    while (model_file >> h_min >> h_max >> w_min >> w_max)
    {
        ObjectSize obj_sz(h_min, h_max, w_min, w_max);
        model_vec.push_back(obj_sz);
    }

    // save sizes to models vector
    models_[model_name] = model_vec;

    return true;
}

ED_REGISTER_PERCEPTION_MODULE(SizeMatcher)
}
}

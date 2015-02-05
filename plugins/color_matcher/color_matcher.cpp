/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#include "color_matcher.h"

#include "ed/measurement.h"
#include <ed/entity.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>


ColorMatcher::ColorMatcher() :
    PerceptionModule("color_matcher"),
    color_table_(ColorNameTable::instance()),  // Init colorname table
    init_success_(false)
{
}

// ---------------------------------------------------------------------------------------------------

ColorMatcher::~ColorMatcher()
{
}

// ----------------------------------------------------------------------------------------------------

void ColorMatcher::configure(tue::Configuration config) {

    if (!config.value("color_table", color_table_path_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'color_table' not found. Using default: " << color_table_path_ << std::endl;

    color_table_path_ = module_path_ + color_table_path_;

    if (!config.value("debug_mode", debug_mode_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'debug_mode' not found. Using default: " << debug_mode_ << std::endl;

    if (!config.value("debug_folder", debug_folder_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'debug_folder' not found. Using default: " << debug_folder_ << std::endl;

    if (debug_mode_)
        cleanDebugFolder(debug_folder_);

    std::cout << "[" << module_name_ << "] " << "Loading color names..." << std::endl;

    if (!color_table_.load_config(color_table_path_)){
        std::cout << "[" << module_name_ << "] " << "Failed loading color names from " << color_table_path_ << std::endl;
        return;
    }

    init_success_ = true;

    std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void ColorMatcher::loadConfig(const std::string& config_path) {

    module_name_ = "color_matcher";
    module_path_ = config_path;

    // default values in case configure(...) is not called!
    debug_folder_ = "/tmp/color_matcher/";
    debug_mode_ = false;
    color_table_path_ = config_path + "/color_names.txt";
}


// ---------------------------------------------------------------------------------------------------


void ColorMatcher::loadModel(const std::string& model_name, const std::string& model_path)
{
    std::string models_folder = model_path.substr(0, model_path.find_last_of("/") - 1); // remove last slash
    models_folder = models_folder.substr(0, models_folder.find_last_of("/"));   // remove color from path

    std::string path = models_folder + "/models/" + model_name +  "/" +  model_name + ".yml";

    if (loadLearning(path, model_name)){
//        std::cout << "[" << module_name_ << "] " << "Loaded colors for " << model_name << std::endl;
    }
    else{
//        std::cout << "[" << kModuleName << "] " << "Couldn not load colors for " << path << "!" << std::endl;
    }
}


// ---------------------------------------------------------------------------------------------------


void ColorMatcher::process(ed::EntityConstPtr e, tue::Configuration& result) const
{
    if (!init_success_)
        return;

    // ---------- PREPARE MEASUREMENT ----------

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->lastMeasurement();
    if (!msr)
        return;

    uint min_x, max_x, min_y, max_y;
    std::map<std::string, double> hypothesis;

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

    // initialize mask
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

    optimizeContourBlur(mask, mask);

    // ---------- PROCESS MEASUREMENT ----------

    // Calculate img color prob
    cv::Mat roi (cropped_image(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y)));
    cv::Mat roi_mask (mask(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y)));

    cv::Mat color_hist;
    std::map<std::string, double> color_amount = getImageColorProbability(roi, roi_mask, color_hist);

    getHypothesis(color_hist, hypothesis);

    // ---------- ASSERT RESULTS ----------

    // create group if it doesnt exist
    if (!result.readGroup("perception_result", tue::OPTIONAL))
    {
        result.writeGroup("perception_result");
    }

    result.writeGroup("color_matcher");

    // assert colors
    if (!color_amount.empty()){
        result.writeArray("colors");
        for (std::map<std::string, double>::const_iterator it = color_amount.begin(); it != color_amount.end(); ++it)
        {
            result.addArrayItem();
            result.setValue("name", it->first);
            result.setValue("value", it->second);
            result.endArrayItem();
        }
        result.endArray();
    }


    // assert hypothesis
    if (!hypothesis.empty()){
        result.writeArray("hypothesis");
        for (std::map<std::string, double>::const_iterator it = hypothesis.begin(); it != hypothesis.end(); ++it)
        {
            result.addArrayItem();
            result.setValue("name", it->first);
            result.setValue("score", it->second);
            result.endArrayItem();
        }
        result.endArray();
    }

    result.endGroup();  // close color_matcher group
    result.endGroup();  // close perception_result group


    // ---------- DEBUG ----------

    if (debug_mode_){
        cv::Mat temp;
        ed::UUID id = ed::Entity::generateID();

        roi.copyTo(temp, roi_mask);

        cv::imwrite(debug_folder_ + id.str() + "_color_matcher_full.png", roi);
        cv::imwrite(debug_folder_ + id.str() + "_color_matcher_masked.png", temp);
    }
}

// ---------------------------------------------------------------------------------------------------

std::map<std::string, double> ColorMatcher::getImageColorProbability(const cv::Mat& img, const cv::Mat& mask, cv::Mat& histogram) const
{
    std::map<std::string, unsigned int> color_count;
    uint pixel_count = 0;

    // Loop over the image
    for(int y = 0; y < img.rows; ++y) {
        for(int x = 0; x < img.cols; ++x) {

            // only use the points covered by the mask
            if (mask.at<unsigned char>(y, x) > 0){
                pixel_count ++;

                // Calculate prob distribution
                const cv::Vec3b& c = img.at<cv::Vec3b>(y, x);

                ColorNamePoint cp((float) c[2],(float) c[1],(float) c[0]);
                std::vector<ColorProbability> probs = color_table_.getProbabilities(cp);

                std::string highest_prob_name;
                float highest_prob = 0;

                for (std::vector<ColorProbability>::iterator it = probs.begin(); it != probs.end(); ++it) {

                    if (it->probability() > highest_prob) {
                        highest_prob = it->probability();
                        highest_prob_name = it->name();
                    }
                }

                // Check if the highest prob name exists in the map
                std::map<std::string, unsigned int>::iterator found_it = color_count.find(highest_prob_name);
                if (found_it == color_count.end()) // init on 1
                    color_count.insert( std::pair<std::string, unsigned int>(highest_prob_name,1) );
                else // +1
                    color_count[highest_prob_name] += 1;

                // Set the color in the image (vis purposes only)

                int r,g,b;
                colorToRGB(stringToColor(highest_prob_name),r,g,b);
//                img.at<cv::Vec3b>(y, x) = cv::Vec3b(b,g,r);       // Paint over the image for debugging
            }
        }
    }

    // initialize histogram
    histogram = cv::Mat::zeros(1, ColorNames::getTotalColorsNum(), CV_32FC1);

    std::map<std::string, double> color_prob;
    for (std::map<std::string, unsigned int>::const_iterator it = color_count.begin(); it != color_count.end(); ++it) {
        color_prob.insert(std::pair<std::string, double>(it->first, (double) it->second/pixel_count));

        histogram.at<float>(colorToInt(it->first)) = (double)it->second/pixel_count;
    }

    return color_prob;
}

// ----------------------------------------------------------------------------------------------------


void ColorMatcher::getHypothesis(cv::Mat& curr_hist, std::map<std::string, double>& hypothesis) const
{

    std::map<std::string, std::vector<std::map<std::string, double> > >::const_iterator model_it;
    std::map<std::string, double>::const_iterator set_it;
    std::string model_name;

    double best_score;

    // iterate through all learned models
    for(model_it = models_colors_.begin(); model_it != models_colors_.end(); ++model_it){
        model_name = model_it->first;
        best_score = 0;

        // iterate through all color sets
        for (uint i = 0; i < model_it->second.size(); i++){
            double score = 0;
            cv::Mat model_hist = cv::Mat::zeros(1, ColorNames::getTotalColorsNum(), CV_32FC1);

            // fill histogram for the current color set of the model
            for(set_it = model_it->second[i].begin(); set_it != model_it->second[i].end(); ++set_it){
                model_hist.at<float>(colorToInt(set_it->first)) = set_it->second;
            }

            // use correlation to compare histograms
            score = cv::compareHist(model_hist, curr_hist, CV_COMP_CORREL);

            if (best_score < score){
                best_score = score;
            }
        }

        // save hypothesis score, 1 is correct, 0 is incorrect, sometimes get negative dont know why
        if (best_score > 0)
            hypothesis.insert(std::pair<std::string, double>(model_name, best_score));
    }
}


// ---------------------------------------------------------------------------------------------------

std::string ColorMatcher::getHighestProbColor(std::map<std::string, double>& map) const
{
    double max = 0;
    std::string max_name;
    for (std::map<std::string, double>::const_iterator it = map.begin(); it != map.end(); ++it)
    {
        if (it->second > max) {
            max = it->second;
            max_name = it->first;
        }
    }
    return max_name;
}

// ----------------------------------------------------------------------------------------------------

void ColorMatcher::optimizeContourBlur(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const{

    mask_orig.copyTo(mask_optimized);

    // blur the contour, also expands it a bit
    for (uint i = 6; i < 18; i = i + 2){
        cv::blur(mask_optimized, mask_optimized, cv::Size( i, i ), cv::Point(-1,-1) );
    }

    cv::threshold(mask_optimized, mask_optimized, 50, 255, CV_THRESH_BINARY);
}

// ----------------------------------------------------------------------------------------------------

void ColorMatcher::cleanDebugFolder(const std::string& folder) const{
    if (system(std::string("mkdir " + folder).c_str()) != 0){
        //printf("\nUnable to create output folder. Already created?\n");
    }
    if (system(std::string("rm " + folder + "*.png").c_str()) != 0){
        //printf("\nUnable to clean output folder \n");
    }
}

// ----------------------------------------------------------------------------------------------------

bool ColorMatcher::loadLearning(std::string path, std::string model_name){
    if (path.empty()){
        std::cout << "[" << module_name_ << "] " << "Empty path!" << path << std::endl;
        return false;
    }else{
        tue::Configuration conf;
        double amount;
        std::string color;
        std::string model_name = "";

        if (conf.loadFromYAMLFile(path)){       // read YAML configuration
            if (conf.readGroup("model")){       // read Model group
                if (!conf.value("name", model_name)){   // read model Name
                    std::cout << "[" << module_name_ << "] " << "Could not find model name!" << std::endl;
                }
                if (conf.readArray("color")){    // read color array
                    std::vector<std::map<std::string, double> > color_sets;

                    while(conf.nextArrayItem()){
                        if (conf.readArray("set")){    // read set array

                            std::map<std::string, double> set;
                            while(conf.nextArrayItem()){
                                color = "";
                                // Iterate through all existing colors, Orange is 0, Black is 10, because...
                                for (ColorNames::Color it_color = ColorNames::Orange; it_color <= ColorNames::Black; ++it_color)
                                {
                                    // read the color and amount
                                    if (conf.value(colorToString(it_color), amount, tue::OPTIONAL)){
                                        color = colorToString(it_color);
                                        break;
                                    }
                                }

                                if (color.empty())
                                    std::cout << "[" << module_name_ << "] " << "Error: Unmatched color name, not good!" << std::endl;

                                // add color and amount to set
                                set.insert(std::pair<std::string, double>(color, amount));
                            }
                            conf.endArray();     // close Set array

                            // save the set
                            color_sets.push_back(set);
                        }
                    }

                    // save the sets for this model
                    models_colors_.insert(std::pair<std::string, std::vector<std::map<std::string, double> > >(model_name, color_sets));

                    conf.endArray();     // close Color array
                }else
                    std::cout << "[" << module_name_ << "] " << "Could not find 'size' group" << std::endl;

                conf.endGroup();    // close Model group
            }else
                std::cout << "[" << module_name_ << "] " << "Could not find 'model' group" << std::endl;
        }else{
//            std::cout << "[" << kModuleName << "] " << "Didn't find configuration file." << std::endl;
            return false;
        }
    }

    return true;
}

ED_REGISTER_PERCEPTION_MODULE(ColorMatcher)

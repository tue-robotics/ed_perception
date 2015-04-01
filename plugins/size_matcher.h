/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: July 2015
*/

#ifndef ED_PERCEPTION_SIZE_MATCHER_H_
#define ED_PERCEPTION_SIZE_MATCHER_H_

#include <ed/perception/module.h>

struct ObjectSize
{
    double min_height;
    double max_height;
    double min_width;
    double max_width;

    ObjectSize(){}

    ObjectSize(double hmin, double hmax, double wmin, double wmax)
    {
        min_height = std::max(hmin, 0.0);
        max_height = hmax;
        min_width = std::max(wmin, 0.0);
        max_width = wmax;
    }

    // Update values
    void updateHMin(double hmin) {min_height = std::max(hmin, 0.0);}
    void updateHMax(double hmax) {max_height = hmax;}
    void updateWMin(double wmin) {min_width = std::max(wmin, 0.0);}
    void updateWmax(double wmax) {max_width = wmax;}
};


// ##################################################################################
// ##################################################################################


class SizeMatcher : public ed::perception::Module
{

public:

    SizeMatcher();

    virtual ~SizeMatcher();

    void loadModel(const std::string& model_name, const std::string& model_path);

    void loadConfig(const std::string& config_path);

    void process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const;

    void configure(tue::Configuration config);

private:

    bool init_success_;

    float small_size_treshold_;
    float medium_size_treshold_;
    float size_diff_threshold_;

    std::map<std::string, std::vector<ObjectSize> > models_;
    std::string	module_name_;    /*!< Name of the module, for output */

    bool loadLearnedModel(std::string path, std::string model_name);
};

#endif

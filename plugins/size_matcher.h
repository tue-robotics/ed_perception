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
    double height;
    double width;

    ObjectSize(){}

    ObjectSize(double width_, double height_)
        : width(width_), height(height_){}


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

    double type_positive_score_;
    double type_negative_score_;

    bool loadLearnedModel(std::string path, std::string model_name);
};

#endif

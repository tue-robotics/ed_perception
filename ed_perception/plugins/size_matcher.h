/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: July 2015
*/

#ifndef ED_PERCEPTION_SIZE_MATCHER_H_
#define ED_PERCEPTION_SIZE_MATCHER_H_

#include <ed/perception/module.h>

// ----------------------------------------------------------------------------------------------------

struct SizeModel
{
    double width_min;
    double width_max;
    double height_min;
    double height_max;
};

// ----------------------------------------------------------------------------------------------------

class SizeMatcher : public ed::perception::Module
{

public:

    SizeMatcher();

    virtual ~SizeMatcher();

    void classify(const ed::Entity& e, const std::string& property, const ed::perception::CategoricalDistribution& prior,
                  ed::perception::ClassificationOutput& output) const;

    void addTrainingInstance(const ed::Entity& e, const std::string& property, const std::string& value);

    void loadRecognitionData(const std::string& path);

    void saveRecognitionData(const std::string& path) const;

private:

    std::map<std::string, SizeModel> models_;

};

#endif

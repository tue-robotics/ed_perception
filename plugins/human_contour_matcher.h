/*
* Author: Luis Fererira
* E-mail: luisffereira@outlook.com
* Date: July 2015
*/

#ifndef ED_PERCEPTION_HUMAN_CONTOUR_MATCHER_H_
#define ED_PERCEPTION_HUMAN_CONTOUR_MATCHER_H_

#include <ed/perception_modules/perception_module.h>
#include "human_classifier.h"

class HumanContourMatcher : public ed::PerceptionModule
{

private:
    HumanClassifier human_classifier_;

    bool init_success_;
    std::string	kModuleName;    /*!< Name of the module, for output */

public:

    HumanContourMatcher();

    virtual ~HumanContourMatcher();

    void loadConfig(const std::string& config_path);

    void process(ed::EntityConstPtr e, tue::Configuration& result) const;

};

#endif

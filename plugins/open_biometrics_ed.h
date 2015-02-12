/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#ifndef ED_PERCEPTION_OPEN_BR_ED_H_
#define ED_PERCEPTION_OPEN_BR_ED_H_

#include <ed/perception_modules/perception_module.h>

// OpenCV includes
#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"

class OpenBrEd : public ed::PerceptionModule
{

/*
 * ###########################################
 *  				PRIVATE
 * ###########################################
 */
private:

    // module configuration
    mutable bool init_success_;
    bool debug_mode_;            /*!< Enable debug mode */
    std::string	module_name_;    /*!< Name of the module, for output */
    std::string	module_path_;    /*!< Name of the module, for output */
    std::string debug_folder_;   /*!< Path of the debug folder */


/*
* ###########################################
*  				    PUBLIC
* ###########################################
*/
public:

    OpenBrEd();

    virtual ~OpenBrEd();

    void loadConfig(const std::string& config_path);

    void process(ed::EntityConstPtr e, tue::Configuration& result) const;

    void configure(tue::Configuration config);

};

#endif

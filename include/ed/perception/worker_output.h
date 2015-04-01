#ifndef ED_PERCEPTION_WORKER_OUTPUT_H_
#define ED_PERCEPTION_WORKER_OUTPUT_H_

#include <tue/config/configuration.h>
#include "ed/perception/categorical_distribution.h"

namespace ed
{
namespace perception
{

struct WorkerOutput
{
    CategoricalDistribution type_update;
    tue::Configuration data;
};

} // end namespace ed

} // end namespace perception

#endif

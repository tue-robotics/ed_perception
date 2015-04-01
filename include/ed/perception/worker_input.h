#ifndef ED_PERCEPTION_WORKER_INPUT_H_
#define ED_PERCEPTION_WORKER_INPUT_H_

#include <ed/types.h>
#include "ed/perception/categorical_distribution.h"

namespace ed
{
namespace perception
{

struct WorkerInput
{
    ed::EntityConstPtr entity;
    CategoricalDistribution type_distribution;
};

} // end namespace ed

} // end namespace perception

#endif

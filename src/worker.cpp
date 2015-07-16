#include "ed/perception/worker.h"
#include "ed/perception/module.h"

#include <ed/entity.h>
#include <ed/measurement.h>

#include <ed/error_context.h>

namespace ed
{

namespace perception
{

// ----------------------------------------------------------------------------------------------------

Worker::Worker(const std::vector<std::string>& model_list, double type_persistence, double unknown_probability_prior)
    : model_list_(model_list), type_persistence_(type_persistence), unknown_probability_prior_(unknown_probability_prior),
      t_last_processing(0), state_(IDLE), signal_stop_(false)
{
}

// ----------------------------------------------------------------------------------------------------

Worker::~Worker()
{
    processing_thread_.join(); // Warning: this will block
}

// ----------------------------------------------------------------------------------------------------

double Worker::timestamp() const
{
    if (input_.entity)
    {
        MeasurementConstPtr msr = input_.entity->lastMeasurement();
        if (msr)
            return msr->timestamp();
    }
    return 0;
}

// ----------------------------------------------------------------------------------------------------

void Worker::start()
{
    if (!input_.entity)
        return;

    state_ = RUNNING;
    processing_thread_ = boost::thread(boost::bind(&Worker::run, this));
}

// ----------------------------------------------------------------------------------------------------

void Worker::stop()
{

}

// ----------------------------------------------------------------------------------------------------

void Worker::run()
{
//    if (input_.entity->pose().t.x < 0.94)
//        return;

    signal_stop_ = false;

    // Reset from possible previous time
    output_.data = tue::Configuration();

    if (input_.type_distribution.empty() || type_persistence_ == 0)
    {
        input_.type_distribution.setUnknownScore(unknown_probability_prior_);

        // Add all possible model types to the type distribution
        for(std::vector<std::string>::const_iterator it = model_list_.begin(); it != model_list_.end(); ++it)
            input_.type_distribution.setScore(*it, (1.0 - unknown_probability_prior_) / model_list_.size());
    }
    else
    {
        input_.type_distribution.waterDown(1 - type_persistence_);
    }

    // Do the actual processing
    for(std::vector<boost::shared_ptr<Module> >::const_iterator it = modules_.begin(); it != modules_.end(); ++it)
    {
        // Clear type distribution update
        output_.type_update = CategoricalDistribution();

        std::string context_msg = "Perception module '" + (*it)->name() + "', entity '" + input_.entity->id().str() + "'";
        ed::ErrorContext errc(context_msg.c_str());
        (*it)->process(input_, output_);

        // Update total type distribution
        input_.type_distribution.update(output_.type_update);

        if (signal_stop_)
        {
            state_ = STOPPED;
            return;
        }
    }

    // Set state to DONE
    state_ = DONE;
}

}

}

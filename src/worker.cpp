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

Worker::Worker() : t_last_processing(0), state_(IDLE)
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
    if (entity_)
    {
        MeasurementConstPtr msr = entity_->lastMeasurement();
        if (msr)
            return msr->timestamp();
    }
    return 0;
}

// ----------------------------------------------------------------------------------------------------

void Worker::start()
{
    if (!entity_)
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
    // Reset from possible previous time
    result_ = tue::config::DataPointer();

    tue::Configuration rw(result_);

    WorkerInput input;
    input.entity = entity_;

    WorkerOutput output;
    output.data = rw;

    // Do the actual processing
    for(std::vector<boost::shared_ptr<Module> >::const_iterator it = modules_.begin(); it != modules_.end(); ++it)
    {
        std::string context_msg = "Perception module '" + (*it)->name() + "', entity '" + entity_->id().str() + "'";
        ed::ErrorContext errc(context_msg.c_str());
        (*it)->process(input, output);
    }

    // Set state to DONE
    state_ = DONE;
}

}

}

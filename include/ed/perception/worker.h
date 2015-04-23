#ifndef ED_PERCEPTION_WORKER_H_
#define ED_PERCEPTION_WORKER_H_

#include <boost/thread.hpp>
#include "ed/perception/module.h"

namespace ed
{

namespace perception
{

class Worker
{

public:

    enum WorkerState {
        RUNNING,
        DONE,
        IDLE,
        STOPPED
    };

    Worker(const std::vector<std::string>& model_list, double type_persistence);

    virtual ~Worker();

    const char* stateString() const
    {
        static const char* state_strs[] = { "running", "done", "idle", "stopped" };
        return state_strs[state_];
    }

    void start();

    void stop();

    void join() { processing_thread_.join(); }

    bool isRunning() const { return state_ == RUNNING; }

    bool isDone() const { return state_ == DONE; }

    bool isIdle() const { return state_ == IDLE; }

    bool isStopped() const { return state_ == STOPPED; }

    void setIdle() { state_ = IDLE; }

    void signalStop() { signal_stop_ = true; }

    inline void setPerceptionModules(const std::vector<boost::shared_ptr<Module> >& modules)
    {
        modules_ = modules;
    }

    inline void setEntity(const EntityConstPtr& e)
    {
        input_.entity = e;
    }

    double timestamp() const;

    inline tue::config::DataConstPointer getResult() const { return output_.data.data(); }

    inline const CategoricalDistribution& getTypeDistribution() const { return input_.type_distribution; }

    double t_last_processing;

    bool signal_stop_;

protected:

    // List of possible object types
    std::vector<std::string> model_list_;

    double type_persistence_;

    WorkerState state_;

    boost::thread processing_thread_;

    std::vector<boost::shared_ptr<Module> > modules_;

    WorkerInput input_;

    WorkerOutput output_;

//    tue::config::DataPointer result_;

    void run();

};

}

}

#endif

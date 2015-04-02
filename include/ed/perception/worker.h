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
        IDLE
    };

    Worker(const std::vector<std::string>& model_list);

    virtual ~Worker();

    void start();

    void stop();

    void join() { processing_thread_.join(); }

    bool isRunning() const { return state_ == RUNNING; }

    bool isDone() const { return state_ == DONE; }

    bool isIdle() const { return state_ == IDLE; }

    void setIdle() { state_ = IDLE; }

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

protected:

    // List of possible object types
    std::vector<std::string> model_list_;

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

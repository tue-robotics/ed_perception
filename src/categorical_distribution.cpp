#include "ed/perception/categorical_distribution.h"

#include <ed/logging.h>

#include <assert.h>

namespace ed
{
namespace perception
{

// ----------------------------------------------------------------------------------------------------

CategoricalDistribution::CategoricalDistribution() : unknown_score_(0)
{
}

// ----------------------------------------------------------------------------------------------------

CategoricalDistribution::~CategoricalDistribution()
{
}

// ----------------------------------------------------------------------------------------------------

void CategoricalDistribution::setScore(const std::string& label, double score)
{
    assert(score >= 0);

    if (pmf_.empty() && unknown_score_ == 0 && score <= 1.0)
        unknown_score_ = 1.0 - score;

    pmf_[label] = score;
}

// ----------------------------------------------------------------------------------------------------

double CategoricalDistribution::getScore(const std::string& label) const
{
    std::map<std::string, double>::const_iterator it = pmf_.find(label);
    if (it != pmf_.end())
        return it->second;

    return unknown_score_;
}

// ----------------------------------------------------------------------------------------------------

void CategoricalDistribution::setUnknownScore(double score)
{
    unknown_score_ = score;
}

// ----------------------------------------------------------------------------------------------------

void CategoricalDistribution::normalize()
{
    double total_score = unknown_score_;
    for(std::map<std::string, double>::const_iterator it = pmf_.begin(); it != pmf_.end(); ++it)
        total_score += it->second;

    unknown_score_ /= total_score;

    for(std::map<std::string, double>::iterator it = pmf_.begin(); it != pmf_.end(); ++it)
        it->second /= total_score;
}

// ----------------------------------------------------------------------------------------------------

void CategoricalDistribution::update(const CategoricalDistribution& other)
{
    // Make sure all labels in 'other' exist in this
    for(std::map<std::string, double>::const_iterator it = other.pmf_.begin(); it != other.pmf_.end(); ++it)
    {
        if (pmf_.find(it->first) == pmf_.end())
        {
            ed::log::error() << "While updating Categorical Distribution: unknown label '" << it->first << "'." << std::endl;
            assert(pmf_.find(it->first) == pmf_.end());
        }
    }

    // Update
    for(std::map<std::string, double>::iterator it = pmf_.begin(); it != pmf_.end(); ++it)
        it->second *= other.getScore(it->first);

    normalize();
}


} // end namespace ed

} // end namespace perception


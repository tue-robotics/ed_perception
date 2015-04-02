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

bool CategoricalDistribution::getScore(const std::string& label, double& score) const
{
    std::map<std::string, double>::const_iterator it = pmf_.find(label);
    if (it == pmf_.end())
        return false;

    score = it->second;
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool CategoricalDistribution::getMaximum(std::string& label, double& score) const
{
    if (pmf_.empty())
        return false;

    double best_score = 0;

    for(std::map<std::string, double>::const_iterator it = pmf_.begin(); it != pmf_.end(); ++it)
    {
        if (it->second > best_score)
        {
            label = it->first;
            best_score = it->second;
        }
    }

    if (best_score == 0)
        return false;

    score = best_score;
    return true;
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
    if (other.pmf_.empty())
        return;

    // Make sure all labels in 'other' exist in this
    for(std::map<std::string, double>::const_iterator it = other.pmf_.begin(); it != other.pmf_.end(); ++it)
    {
        if (pmf_.find(it->first) == pmf_.end())
        {
            ed::log::error() << "While updating Categorical Distribution: unknown label '" << it->first << "'." << std::endl;
            assert(pmf_.find(it->first) != pmf_.end());
        }
    }

    // Update
    for(std::map<std::string, double>::iterator it = pmf_.begin(); it != pmf_.end(); ++it)
    {
        double score;
        if (!other.getScore(it->first, score))
            score = other.unknown_score_ / (pmf_.size() - other.pmf_.size());

        it->second *= score;
    }

    unknown_score_ *= other.unknown_score_;

    normalize();
}

// ----------------------------------------------------------------------------------------------------

std::ostream& operator<< (std::ostream& out, const CategoricalDistribution& c)
{
    out << "[";

    if (!c.pmf_.empty())
    {
        std::map<std::string, double>::const_iterator it = c.pmf_.begin();
        out << it->first << ": " << it->second;
        ++it;
        for(; it != c.pmf_.end(); ++it)
            out << ", " << it->first << ": " << it->second;
    }

    out << " | unknown = " << c.unknown_score_ << " ]";

    return out;
}

} // end namespace ed

} // end namespace perception


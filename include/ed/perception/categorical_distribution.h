#ifndef ED_PERCEPTION_CATEGORICAL_DISTRIBUTION_H_
#define ED_PERCEPTION_CATEGORICAL_DISTRIBUTION_H_

#include <map>
#include <string>

#include <ostream>

namespace ed
{
namespace perception
{

class CategoricalDistribution
{

public:

    CategoricalDistribution();

    ~CategoricalDistribution();

    void setScore(const std::string& label, double score);

    bool getScore(const std::string& label, double& score) const;

    bool getMaximum(std::string& label, double& score) const;

    void setUnknownScore(double score);

    double getUnknownScore() const { return unknown_score_; }

    void normalize();

    void update(const CategoricalDistribution& other);

    bool empty() const { return pmf_.empty(); }

    void waterDown(double factor);

    friend std::ostream& operator<< (std::ostream& out, const CategoricalDistribution& c);

private:

    double unknown_score_;

    std::map<std::string, double> pmf_;

};


} // end namespace ed

} // end namespace perception

#endif

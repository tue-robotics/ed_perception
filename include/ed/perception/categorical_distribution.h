#ifndef ED_PERCEPTION_CATEGORICAL_DISTRIBUTION_H_
#define ED_PERCEPTION_CATEGORICAL_DISTRIBUTION_H_

#include <map>
#include <string>

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

    double getScore(const std::string& label) const;

    void setUnknownScore(double score);

    void normalize();

    void update(const CategoricalDistribution& other);

private:

    double unknown_score_;

    std::map<std::string, double> pmf_;

};

} // end namespace ed

} // end namespace perception

#endif

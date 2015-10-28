#ifndef CONFUSIONMATRIX_H
#define CONFUSIONMATRIX_H

#include <vector>
#include <string>
#include <opencv/cv.h>
#include "ed/perception/categorical_distribution.h"


class ConfusionMatrix
{
public:
    ConfusionMatrix();

    cv::Mat toCvMat();

    void addResult(const ed::perception::CategoricalDistribution& dstr, const std::string& cat);

    int size();

//    int getMaximum();

    std::vector<std::string> getOptions();

    int at(int result, int truth );

    void show();

    int resize_factor;

private:

//    std::vector<int> mat_;
    std::vector<std::vector<int> > mat_;
    std::vector<std::string> options_;
    std::map<std::string,int> option_indices_;
    std::vector<int> unknowns;
//    int maximum_;
};

#endif // CONFUSIONMATRIX_H

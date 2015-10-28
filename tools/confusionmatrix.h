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

    void addResult(const ed::perception::CategoricalDistribution& dstr, const std::string& cat);

    void show();

    int at(int result, int truth ) const;

    std::vector<std::string> getOptions() const;

    cv::Mat getImage() const {return img_;}

private:
    friend void onMouse(int event, int x, int y, int flags, void* param);

    void drawImage_();

    int resize_factor_;
    cv::Mat img_;
    std::vector<std::vector<int> > mat_;
    std::vector<std::string> options_;
    std::map<std::string,int> option_indices_;
    std::vector<int> unknowns;
};

#endif // CONFUSIONMATRIX_H

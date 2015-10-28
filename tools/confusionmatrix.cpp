#include "confusionmatrix.h"
#include <opencv/highgui.h>


// ----------------------------------------------------------------------------------------------------

ConfusionMatrix::ConfusionMatrix(): resize_factor_(20) {}

// ----------------------------------------------------------------------------------------------------

void onMouse(int event, int x, int y, int flags, void* param)
{
    ConfusionMatrix* cm = (ConfusionMatrix*) param;

    char text[100];
    char counttext[20];

    cv::Mat img2 = cm->img_.clone();

    int resulti = x/cm->resize_factor_;
    int gtruthi = y/cm->resize_factor_;

    std::string result = cm->getOptions()[resulti];
    std::string gtruth = cm->getOptions()[gtruthi];

    int count = cm->at(resulti,gtruthi);

    sprintf(text, "Perc. result: %s, truth: %s.", result.c_str(), gtruth.c_str());
    sprintf(counttext, "Count=%i", count);
    cv::putText(img2, text, cv::Point(cm->resize_factor_+5, 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0,255,0));
    cv::putText(img2, counttext, cv::Point(cm->resize_factor_+5, 35), cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0,255,0));
    cv::imshow("Confusion matrix", img2);
}

// ----------------------------------------------------------------------------------------------------

void ConfusionMatrix::addResult(const ed::perception::CategoricalDistribution& dstr, const std::string& cat)
{
    for ( std::map<std::string,double>::const_iterator it = dstr.values().begin(); it != dstr.values().end(); it++ )
    {
        if ( option_indices_.find(it->first) == option_indices_.end() )
        {
            option_indices_[it->first] = options_.size();
            options_.push_back(it->first);
            unknowns.resize(options_.size(),0);
        }
    }
    if ( option_indices_.find(cat) == option_indices_.end() )
    {
        option_indices_[cat] = options_.size();
        options_.push_back(cat);
        unknowns.resize(options_.size(),0);
    }

    std::string label;
    double score;
    int labeli, cati;

    dstr.getMaximum(label,score);

    // Get option index of ground truth
    cati   = option_indices_[cat];

    // If unknown is higher than other scores, only count the unknown
    if ( dstr.getUnknownScore() > score )
        unknowns[cati]++;
    else
    {
        // Get option index of perception result
        labeli = option_indices_[label];

        // Resize confusion matrix if necessary
        if ( cati >= mat_.size() )
            mat_.resize(cati+1);
        if ( labeli >= mat_[cati].size() )
            mat_[cati].resize(labeli+1,0);

        // Add one to the right matrix element
        mat_[cati][labeli]++;
    }

}

// ----------------------------------------------------------------------------------------------------

void ConfusionMatrix::show()
{
    cv::namedWindow("Confusion matrix");
    cv::setMouseCallback("Confusion matrix", onMouse, this);
    drawImage_();
    cv::imshow("Confusion matrix", img_);
    cv::waitKey();
}

// ----------------------------------------------------------------------------------------------------

int ConfusionMatrix::at(int result, int truth ) const
{
    if ( truth >= mat_.size() )
        return 0;

    if ( result == options_.size() )
        return unknowns[truth];
    else if ( result >= mat_[truth].size() )
        return 0;
    else
        return mat_[truth][result];
}

// ----------------------------------------------------------------------------------------------------

std::vector<std::string> ConfusionMatrix::getOptions() const
{
    std::vector<std::string> options(options_);
    options.push_back("unknown");
    return options;
}

// ----------------------------------------------------------------------------------------------------

void ConfusionMatrix::drawImage_()
{
    int column, row = 0;

    resize_factor_ = 300/options_.size();

    img_ = cv::Mat(resize_factor_ * options_.size(), resize_factor_* (options_.size() + 1), CV_8UC3, cv::Scalar(0, 0, 0));

    for ( std::vector<std::vector<int> >::iterator row_it = mat_.begin(); row_it != mat_.end(); row_it++ )
    {
        column = 0;

        int total_count = unknowns[row];
        for (std::vector<int>::iterator it = row_it->begin(); it != row_it->end(); it++)
            total_count += *it;

        double f = 1.0 / total_count;

        if ( unknowns[row] > 0 )
        {
            double v = f * unknowns[row];
            cv::Point2d center = resize_factor_ * cv::Point2d(0.5 + options_.size(), 0.5 + row);
            cv::Point2d half_size = (resize_factor_ * v / 2) * cv::Point2d(1, 1);
            cv::rectangle(img_, center - half_size, center + half_size, cv::Scalar(0, 0, 255), CV_FILLED);
        }

        for (std::vector<int>::iterator it = row_it->begin(); it != row_it->end(); it++ )
        {
            if ( *it > 0 )
            {
                double v = f * (*it);

                cv::Point2d center = resize_factor_ * cv::Point2d(0.5 + column, 0.5 + row);
                cv::Point2d half_size = (resize_factor_ * v / 2) * cv::Point2d(1, 1);

                cv::Scalar color(0, 0, 255);
                if (row == column)
                    color = cv::Scalar(0, 255, 0);

                cv::rectangle(img_, center - half_size, center + half_size, color, CV_FILLED);
            }
            column++;
        }
        row++;
    }

    cv::Scalar line_color(51, 51, 51);

    for ( int i = 0; i <= options_.size(); i++ )
    {
        // Horizontal line
        cv::line(img_,cv::Point(0,resize_factor_*i),cv::Point(resize_factor_*(options_.size()+1),resize_factor_*i), line_color);
        // Vertical line
        cv::line(img_,cv::Point(resize_factor_*i,0),cv::Point(resize_factor_*i,resize_factor_*(options_.size()+1)), line_color);
    }
}

// ----------------------------------------------------------------------------------------------------



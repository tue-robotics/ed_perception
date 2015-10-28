#include "confusionmatrix.h"
#include <opencv/highgui.h>


// ----------------------------------------------------------------------------------------------------

ConfusionMatrix::ConfusionMatrix(): resize_factor(20)
{
}

// ----------------------------------------------------------------------------------------------------

cv::Mat ConfusionMatrix::toCvMat()
{
    int column = 0, row = 0;

    cv::Mat mat = cv::Mat(options_.size(), options_.size()+1, CV_64FC3, cv::Scalar(0.0,0.0,0.0));

    for ( std::vector<std::vector<int> >::iterator row_it = mat_.begin(); row_it != mat_.end(); row_it++ )
    {
        if ( unknowns[row] > 0 )
            mat.at<cv::Vec3d>(row,size())[2] = 1.0;

        for (std::vector<int>::iterator it = row_it->begin(); it != row_it->end(); it++ )
        {
            if ( *it > 0 )
            {
                if ( row == column )
                    mat.at<cv::Vec3d>(row,column)[1] = 1.0;
                else
                    mat.at<cv::Vec3d>(row,column)[2] = 1.0;
            }
            column++;
        }
        row++;
    }

    cv::Mat dst;

    cv::resize(mat,dst,cv::Size(0,0),resize_factor,resize_factor,cv::INTER_NEAREST);

    for ( int i = 0; i < options_.size(); i++ )
    {
        cv::line(dst,cv::Point(0,resize_factor*i),cv::Point(resize_factor*options_.size(),resize_factor*i),cv::Scalar(.2,.2,.2));
        cv::line(dst,cv::Point(resize_factor*i,0),cv::Point(resize_factor*i,resize_factor*options_.size()),cv::Scalar(.2,.2,.2));
    }

    return dst;
}

// ----------------------------------------------------------------------------------------------------

void ConfusionMatrix::addResult(const ed::perception::CategoricalDistribution& dstr, const std::string& cat)
{
    for ( std::map<std::string,double>::const_iterator it = dstr.values().begin(); it != dstr.values().begin(); it++ )
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

    std::cout << "Ground truth: " << cat << "; perception result: " << label << std::endl;

    labeli = option_indices_[label];
    cati   = option_indices_[cat];

    if ( options_.size() > mat_.size() )
        mat_.resize(options_.size());
    if ( options_.size() > mat_[cati].size() )
        mat_[cati].resize(options_.size(),0);

    if ( dstr.getUnknownScore() > score )
        labeli = 0;

    mat_[cati][labeli]++;

}

// ----------------------------------------------------------------------------------------------------

int ConfusionMatrix::size()
{
    return options_.size();
}

// ----------------------------------------------------------------------------------------------------

std::vector<std::string> ConfusionMatrix::getOptions()
{
    std::vector<std::string> options(options_);
    options.push_back("unknown");
    return options;
}

// ----------------------------------------------------------------------------------------------------

int ConfusionMatrix::at(int result, int truth )
{
    return mat_[truth][result];
}

// ----------------------------------------------------------------------------------------------------

void onMouse(int event, int x, int y, int flags, void* param)
{
    ConfusionMatrix* cm = (ConfusionMatrix*) param;

    std::cout << "print 1" << std::endl;

    cv::Mat mat = cm->toCvMat();

    std::cout << "print 2" << std::endl;

    char text[100];
    char counttext[20];
    cv::Mat img2;

    img2 = mat.clone();

    int resulti = x/cm->resize_factor;
    int gtruthi = y/cm->resize_factor;

    std::string result = cm->getOptions()[resulti];
    std::string gtruth = cm->getOptions()[gtruthi];

    int count = cm->at(resulti,gtruthi);

    sprintf(text, "Perception result: %s, ground truth: %s.", result.c_str(), gtruth.c_str());
    sprintf(counttext, "Count=%i", count);
    cv::putText(img2, text, cv::Point(45,15), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0));
    cv::putText(img2, counttext, cv::Point(45,35), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0));
    cv::imshow("Confusion matrix", img2);
}

// ----------------------------------------------------------------------------------------------------

void ConfusionMatrix::show()
{
    cv::namedWindow("Confusion matrix");
    cv::setMouseCallback("Confusion matrix", onMouse, this);
    cv::imshow("Confusion matrix", toCvMat());
    cv::waitKey();
}

// ----------------------------------------------------------------------------------------------------



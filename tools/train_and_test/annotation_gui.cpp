#include <QApplication>
#include <QInputDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QDialogButtonBox>

#include "image_crawler.h"

#include <opencv2/highgui/highgui.hpp>
#include <rgbd/Image.h>

// ---------------------------------------------------------------------------------------------

int CIRCLE_RADIUS = 6;
double FONT_SIZE = 1.0;
std::string WINDOW_NAME = "annotation_gui";

// ---------------------------------------------------------------------------------------------

void redraw(const AnnotatedImage& image, const std::string& filename, int index, int total)
{
    const cv::Mat& img = image.image->getRGBImage();
    cv::Mat draw_img = img.clone();

    std::stringstream ss; ss << "(" << (index + 1) << "/" << total << ") " << filename;

    cv::rectangle(draw_img, cv::Point(0,0), cv::Point(img.cols,16), CV_RGB(0,0,0), CV_FILLED);
    cv::putText(draw_img, ss.str().c_str(), cv::Point2i(0, 12), 0, 0.4, cv::Scalar(255,255,255), 1);

    for (std::vector<Annotation>::const_iterator it = image.annotations.begin(); it != image.annotations.end(); ++it)
    {
        const Annotation& a = *it;

        int x = a.px * img.cols;
        int y = a.py * img.rows;

        cv::Scalar text_color(255, 0, 255);
        if (a.is_supporting)
            text_color = cv::Scalar(255, 255, 0);

        cv::circle(draw_img, cv::Point2i(x, y), CIRCLE_RADIUS, cv::Scalar(255,0,0), CV_FILLED);
        cv::putText(draw_img, a.label.c_str(), cv::Point2i(x+15,y), 0, FONT_SIZE, text_color, 2);
    }
    cv::imshow(WINDOW_NAME, draw_img);
}

// ---------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    std::string path = ".";
    if (argc > 1)
    {
        path = argv[1];
    }
    else
    {
        QMessageBox mbox;
        mbox.setText("No path specified, I will take the current working directory ...");
        mbox.exec();
    }

    ImageCrawler crawler;
    crawler.setPath(path);

    AnnotatedImage image;

    if (!crawler.next(image))
    {
        QMessageBox mbox;
        mbox.setText("No meta-data files found.");
        mbox.exec();
        return 1;
    }

    while(true)
    {
        redraw(image, crawler.filename(), crawler.index(), crawler.filenames().size());
        char key = cv::waitKey();

        if (key == 81) // left
        {
            crawler.previous(image);
        }
        else if (key == 83) // right
        {
            crawler.next(image);
        }
        else if (key == 'q')
        {
            break;
        }
    }

    return 0;
}

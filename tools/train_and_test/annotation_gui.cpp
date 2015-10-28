#include <QApplication>
#include <QInputDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QDialogButtonBox>

#include "image_crawler.h"

#include <opencv2/highgui/highgui.hpp>
#include <rgbd/Image.h>

class GUI
{

public:

    GUI() : CIRCLE_RADIUS(6), FONT_SIZE(1.0), WINDOW_NAME("Annotation GUI"), i_possible_types(0)
    {
    }

    // ---------------------------------------------------------------------------------------------

    int CIRCLE_RADIUS;
    double FONT_SIZE;
    std::string WINDOW_NAME;

    ImageCrawler crawler;
    AnnotatedImage image;

    std::string typed;
    std::set<std::string> types;
    std::vector<std::string> possible_types;
    int i_possible_types;

    // ---------------------------------------------------------------------------------------------

    void redraw()
    {
        const cv::Mat& img = image.image->getRGBImage();
        cv::Mat draw_img = img.clone();

        std::stringstream ss; ss << "(" << (crawler.index() + 1) << "/" << crawler.filenames().size() << ") " << crawler.filename();

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

        if (!possible_types.empty())
        {
            int n = 3;
            int i_min = std::max<int>(0, i_possible_types - n + 1);
            int i_max = std::min<int>(possible_types.size(), i_min + n);

            for(int i = i_min; i < i_max; ++i)
            {
                cv::Scalar text_color(255, 0, 0);
                if (i == i_possible_types)
                    text_color = cv::Scalar(255, 255, 0);

                cv::putText(draw_img, possible_types[i], cv::Point2i(20, 40 + 30 * (i - i_min)), 0, FONT_SIZE, text_color, 2);
            }
        }

        cv::imshow(WINDOW_NAME, draw_img);
    }

    // ---------------------------------------------------------------------------------------------

    void updatePossibleTypes()
    {
        i_possible_types = 0;

        possible_types.clear();
        if (typed.empty())
            return;

        possible_types.push_back(typed);
        for(std::set<std::string>::const_iterator it = types.begin(); it != types.end(); ++it)
        {
            const std::string& t = *it;
            if (t.size() >= typed.size() && t.substr(0, typed.size()) == typed)
                possible_types.push_back(t);
        }
    }

    // ---------------------------------------------------------------------------------------------

    int run(const std::string path)
    {
        crawler.setPath(path);

        if (!crawler.next(image))
        {
            QMessageBox mbox;
            mbox.setText("No meta-data files found.");
            mbox.exec();
            return 1;
        }

        std::string alpha = "abcdefghijklmnopqrstuvwxyz1234567890_-+./";

        while(true)
        {
            for(std::vector<Annotation>::const_iterator it = image.annotations.begin(); it != image.annotations.end(); ++it)
                types.insert(it->label);



            redraw();
            char key = cv::waitKey();

            if (key == 81) // left
            {
                crawler.previous(image);
            }
            else if (key == 83) // right
            {
                crawler.next(image);
            }
            else if (key == 82) // up
            {
                if (i_possible_types > 0)
                    --i_possible_types;
            }
            else if (key == 84) // down
            {
                if (i_possible_types + 1 < possible_types.size())
                    ++i_possible_types;
            }
            else if (key == 27) // ESC
            {
                break;
            }
            else if (key == 8) // Backspace
            {
                // If not empty, remove last character
                if (!typed.empty())
                {
                    typed = typed.substr(0, typed.size() - 1);
                    updatePossibleTypes();
                }
            }
            else if (key == 10) // Enter
            {
//                typed.clear();
//                updatePossibleTypes();
            }
            else if (key == 9) // Tab
            {

            }
            else if (alpha.find(key) != std::string::npos)
            {
                typed += key;
                updatePossibleTypes();
            }
        }

        return 0;
    }
};

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

    GUI gui;
    return gui.run(path);
}

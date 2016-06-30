//#include <QApplication>
//#include <QInputDialog>
//#include <QMessageBox>
//#include <QPushButton>
//#include <QDialogButtonBox>

#include "image_crawler.h"

#include <opencv2/highgui/highgui.hpp>
#include <rgbd/Image.h>
#include <ed/entity.h>
#include <ed/measurement.h>

// ---------------------------------------------------------------------------------------------

void mouseCallback(int event, int x, int y, int flags, void* userdata);

// ---------------------------------------------------------------------------------------------

class GUI
{

public:

    GUI() : CIRCLE_RADIUS(6), FONT_SIZE(1.0), WINDOW_NAME("Annotation GUI"), i_possible_types(0), image_changed(false)
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
    std::string selected_type;

    bool image_changed;

    // ---------------------------------------------------------------------------------------------

    void redraw()
    {
        const cv::Mat& img = image.image->getRGBImage();
        cv::Mat draw_img = img.clone();

        std::stringstream ss; ss << "(" << (crawler.index() + 1) << "/" << crawler.filenames().size() << ") " << crawler.filename();

        int i_entity = 1;
        cv::Mat segm_img(draw_img.rows, draw_img.cols, CV_32SC1, cv::Scalar(0));
        for(std::vector<ed::EntityConstPtr>::const_iterator it = image.entities.begin(); it != image.entities.end(); ++it)
        {
            const ed::EntityConstPtr& e = *it;
            ed::MeasurementConstPtr m = e->bestMeasurement();
            if (!m)
                continue;

            cv::Point p_min(img.cols, img.rows);
            cv::Point p_max(0, 0);

            const ed::ImageMask& mask = m->imageMask();
            for(ed::ImageMask::const_iterator it = mask.begin(img.cols); it != mask.end(); ++it)
            {
                const cv::Point2i& p = *it;
                segm_img.at<int>(p) = i_entity;
                p_min.x = std::min(p_min.x, p.x);
                p_min.y = std::min(p_min.y, p.y);
                p_max.x = std::max(p_max.x, p.x);
                p_max.y = std::max(p_max.y, p.y);
            }

            cv::rectangle(draw_img, p_min, p_max, cv::Scalar(255, 255, 255), 2);
            cv::rectangle(draw_img, p_min - cv::Point(2, 2), p_max + cv::Point(2, 2), cv::Scalar(0, 0, 0), 2);
            cv::rectangle(draw_img, p_min + cv::Point(2, 2), p_max - cv::Point(2, 2), cv::Scalar(0, 0, 0), 2);

            ++i_entity;
        }

        for(int y = 0; y < img.rows; ++y)
        {
            for(int x = 0; x < img.cols; ++x)
            {
                int i = segm_img.at<int>(y, x);
                if (i < 1)
                    draw_img.at<cv::Vec3b>(y, x) = 0.5 * draw_img.at<cv::Vec3b>(y, x);
            }
        }

        ss << "    (area: " << image.area_name << ")";

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

        if (!selected_type.empty())
        {
            cv::putText(draw_img, selected_type, cv::Point2i(20, 50), 0, FONT_SIZE, cv::Scalar(100, 100, 255), 2);
        }
        else if (!possible_types.empty())
        {
            int n = 3;
            int i_min = std::max<int>(0, i_possible_types - n + 1);
            int i_max = std::min<int>(possible_types.size(), i_min + n);

            for(int i = i_min; i < i_max; ++i)
            {
                cv::Scalar text_color(255, 0, 0);
                if (i == i_possible_types)
                    text_color = cv::Scalar(255, 255, 0);

                cv::putText(draw_img, possible_types[i], cv::Point2i(20, 50 + 30 * (i - i_min)), 0, FONT_SIZE, text_color, 2);
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
            std::cerr << "No meta-data files found." << std::endl;
            return 1;
        }

        std::string alpha = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890_-+./:";

        //Create a window
        cv::namedWindow(WINDOW_NAME, 1);

        //set the callback function for any mouse event
        cv::setMouseCallback(WINDOW_NAME, mouseCallback, this);

        while(true)
        {
            for(std::vector<Annotation>::const_iterator it = image.annotations.begin(); it != image.annotations.end(); ++it)
                types.insert(it->label);

            redraw();
            char key = cv::waitKey();

            if (key == 81) // left
            {
                if (image_changed)
                    toFile(crawler.filename(), image);

                crawler.previous(image);
            }
            else if (key == 83) // right
            {
                if (image_changed)
                    toFile(crawler.filename(), image);

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
                if (image_changed)
                    toFile(crawler.filename(), image);

                break;
            }
            else if (key == 8) // Backspace
            {
                // If not empty, remove last character
                if (!typed.empty())
                {
                    typed = typed.substr(0, typed.size() - 1);
                    updatePossibleTypes();
                    selected_type.clear();
                }
            }
            else if (key == 10) // Enter
            {
                selected_type = possible_types[i_possible_types];
                typed.clear();
                updatePossibleTypes();
            }
            else if (key == 9) // Tab
            {
                if (image_changed)
                    toFile(crawler.filename(), image);

                while(crawler.next(image, false))
                {
                    // Add labels
                    for(std::vector<Annotation>::const_iterator it = image.annotations.begin(); it != image.annotations.end(); ++it)
                        types.insert(it->label);

                    bool has_support = false;
                    for(std::vector<Annotation>::const_iterator it = image.annotations.begin();
                        it != image.annotations.end(); ++it)
                    {
                        if (it->is_supporting)
                        {
                            has_support = true;
                            break;
                        }
                    }

                    if (!has_support)
                    {
                        // Reload with segmentation
                        crawler.reload(image, true);
                        break;
                    }
                }
            }
            else if (alpha.find(key) != std::string::npos)
            {
                typed += key;
                updatePossibleTypes();
                selected_type.clear();
            }
        }

        return 0;
    }
};

// ---------------------------------------------------------------------------------------------

void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
    GUI* gui = static_cast<GUI*>(userdata);

    double px = (double)x / gui->image.image->getRGBImage().cols;
    double py = (double)y / gui->image.image->getRGBImage().rows;

    if (event == cv::EVENT_LBUTTONDOWN && gui->selected_type == "exclude")
    {
        gui->image.excluded = true;
        std::cout << "Excluding image" << std::endl;
        gui->image_changed = true;
        gui->redraw();
    }
    else if (event == cv::EVENT_LBUTTONDOWN && gui->selected_type.size() > 5 && gui->selected_type.substr(0, 5) == "area:")
    {
        gui->image.area_name = gui->selected_type.substr(5);
        std::cout << "Setting area to '" << gui->image.area_name << "'" << std::endl;
        gui->image_changed = true;
        gui->redraw();
    }
    else if (event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_RBUTTONDOWN)
    {
//        std::cout << px << ", " << py << std::endl;

        std::vector<Annotation>::iterator it_clicked = gui->image.annotations.end();

        // Loop over all annotations and check whether we have clicked one
        for (std::vector<Annotation>::iterator it = gui->image.annotations.begin(); it != gui->image.annotations.end(); ++it)
        {
            Annotation& a = *it;

            int a_x = a.px * gui->image.image->getRGBImage().cols;
            int a_y = a.py * gui->image.image->getRGBImage().rows;

            if (x > a_x - gui->CIRCLE_RADIUS && x < a_x + gui->CIRCLE_RADIUS && y > a_y - gui->CIRCLE_RADIUS && y < a_y + gui->CIRCLE_RADIUS)
            {
                it_clicked = it;
                break;
            }
        }

        if (it_clicked != gui->image.annotations.end() && event == cv::EVENT_RBUTTONDOWN)
        {
            gui->image.annotations.erase(it_clicked);
            gui->image_changed = true;
        }

        if (it_clicked == gui->image.annotations.end() && event == cv::EVENT_LBUTTONDOWN && !gui->selected_type.empty())
        {
            gui->image.annotations.push_back(Annotation(gui->selected_type, px, py));
            gui->types.insert(gui->selected_type);
            gui->image_changed = true;
        }

        gui->redraw();
    }
}

// ---------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    std::string path = ".";
    if (argc > 1)
        path = argv[1];

    GUI gui;
    return gui.run(path);
}

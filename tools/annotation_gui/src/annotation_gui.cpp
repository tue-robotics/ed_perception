#include <QApplication>
#include <QInputDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QDialogButtonBox>

#include <tue/filesystem/crawler.h>

#include <ed/uuid.h>

#include <ed/io/json_reader.h>
#include <ed/io/json_writer.h>
#include <rgbd/serialization.h>
#include <rgbd/Image.h>

#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>

// Meta-data read / write
#include <tue/config/read.h>
#include <tue/config/write.h>
#include <tue/config/reader.h>
#include <tue/config/writer.h>
#include <tue/config/reader_writer.h>
#include <tue/config/data_pointer.h>

int CIRCLE_RADIUS = 6;
double FONT_SIZE = 1.0;

unsigned int INDEX = 0;
unsigned int TOTAL;

std::string WINDOW_NAME = "annotation_gui";

// ---------------------------------------------------------------------------------------------

struct Annotation
{
    Annotation() {}
    Annotation(std::string label, double px, double py) : label(label), px(px), py(py) {}

    std::string label;
    double px;
    double py;
};

// ---------------------------------------------------------------------------------------------

struct AnnotationData
{
    std::string filename;
//    std::string json;
    tue::config::DataPointer meta_data;
    rgbd::ImagePtr image;
    std::vector<Annotation> annotations;

    void clear()
    {
        filename = "";
        image = rgbd::ImagePtr();
        annotations.clear();
        meta_data = tue::config::DataPointer();
    }
} ANNOTATION_INSTANCE;

// ---------------------------------------------------------------------------------------------

void redraw()
{
    const cv::Mat& img = ANNOTATION_INSTANCE.image->getRGBImage();
    cv::Mat draw_img = img.clone();

    std::stringstream ss; ss << "(" << INDEX+1 << "/" << TOTAL << ") " << ANNOTATION_INSTANCE.filename;

    cv::rectangle(draw_img, cv::Point(0,0), cv::Point(img.cols,16), CV_RGB(0,0,0), CV_FILLED);
    cv::putText(draw_img, ss.str().c_str(), cv::Point2i(0, 12), 0, 0.4, cv::Scalar(255,255,255), 1);

    for (std::vector<Annotation>::const_iterator it = ANNOTATION_INSTANCE.annotations.begin(); it != ANNOTATION_INSTANCE.annotations.end(); ++it)
    {
        const Annotation& a = *it;

        int x = a.px * img.cols;
        int y = a.py * img.rows;
        cv::circle(draw_img, cv::Point2i(x, y), CIRCLE_RADIUS, cv::Scalar(255,0,0), CV_FILLED);
        cv::putText(draw_img, a.label.c_str(), cv::Point2i(x+15,y), 0, FONT_SIZE, cv::Scalar(255,0,255), 2);
    }
    cv::imshow(WINDOW_NAME, draw_img);
}

// ----------------------------------------------------------------------------------------------------

void write()
{
    // Temporarily make new data containing the annotations. If we would directly add to the existing meta-data
    // we would add the existing annotations *again*. Now, we will completely overwrite them.
    tue::config::DataPointer data;

    tue::config::Writer w(data);

    w.writeArray("annotations");
    for (std::vector<Annotation>::const_iterator it = ANNOTATION_INSTANCE.annotations.begin(); it != ANNOTATION_INSTANCE.annotations.end(); ++it)
    {
        w.addArrayItem();
        const Annotation& a = *it;
        w.setValue("label", a.label);
        w.setValue("px", a.px);
        w.setValue("py", a.py);
        w.endArrayItem();
    }
    w.endArray();

    // Add the new annotations to the existing meta data (complete overwriting the annotations array)
    ANNOTATION_INSTANCE.meta_data.add(data);

    tue::config::toFile(ANNOTATION_INSTANCE.filename, ANNOTATION_INSTANCE.meta_data, tue::config::JSON, 0);
}

// ----------------------------------------------------------------------------------------------------

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    double px = (double)x / ANNOTATION_INSTANCE.image->getRGBImage().cols;
    double py = (double)y / ANNOTATION_INSTANCE.image->getRGBImage().rows;

    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        bool annotation_clicked = false;
        // Loop over all annotations and check whether we have clicked one
        for (std::vector<Annotation>::iterator it = ANNOTATION_INSTANCE.annotations.begin(); it != ANNOTATION_INSTANCE.annotations.end(); ++it)
        {
            Annotation& a = *it;

            int a_x = a.px * ANNOTATION_INSTANCE.image->getRGBImage().cols;
            int a_y = a.py * ANNOTATION_INSTANCE.image->getRGBImage().rows;

            if (x > a_x - CIRCLE_RADIUS && x < a_x + CIRCLE_RADIUS && y > a_y - CIRCLE_RADIUS && y < a_y + CIRCLE_RADIUS)
            {
                annotation_clicked = true;

                QInputDialog dialog;

                dialog.setLabelText("Modify annotation:");
                dialog.setCancelButtonText("Delete");
                dialog.setTextValue(QString(a.label.c_str()));

                if (dialog.exec() && !dialog.textValue().toStdString().empty())
                {
                    a.label = dialog.textValue().toStdString();
                }
                else
                {
                    // Remove the annotation
                    ANNOTATION_INSTANCE.annotations.erase(it);
                }

                redraw();
                write();

                break;
            }
        }

        if (!annotation_clicked)
        {
            QInputDialog dialog;
            dialog.setLabelText("Label:");
            if (dialog.exec() && !dialog.textValue().toStdString().empty())
            {
                ANNOTATION_INSTANCE.annotations.push_back(Annotation(dialog.textValue().toStdString(), px, py));
                redraw();
                write();
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------

rgbd::ImagePtr readRGBDImage(const std::string& filename)
{
    rgbd::ImagePtr image(new rgbd::Image);

    std::ifstream f_in;
    f_in.open(filename.c_str(), std::ifstream::binary);

    if (!f_in.is_open())
        return rgbd::ImagePtr();

    tue::serialization::InputArchive a_in(f_in);
    rgbd::deserialize(a_in, *image);

    return image;
}

// ---------------------------------------------------------------------------------------------

bool load(const std::string& filename)
{    
    //! 1) Clear current annotation
    ANNOTATION_INSTANCE.clear();


    //! 2) Parse meta-data file
    try
    {
        ANNOTATION_INSTANCE.meta_data = tue::config::fromFile(filename);
    }
    catch (tue::config::ParseException& e)
    {
        std::cout << "Could not open '" << filename << "'.\n\n" << e.what() << std::endl;

//        QMessageBox mbox;
//        mbox.setText(QString::fromStdString("Could not open '" + filename + "'.\n\n" + e.what()));
//        mbox.exec();
        return false;
    }

//    tue::config::toStream(std::cout, ANNOTATION_INSTANCE.meta_data, tue::config::JSON, 2);

    ANNOTATION_INSTANCE.filename = filename;

    tue::config::ReaderWriter r(ANNOTATION_INSTANCE.meta_data);

    //! 3) Read image
    std::string rgbd_filename;
    if (r.value("rgbd_filename", rgbd_filename))
    {
        std::string base_path = tue::filesystem::Path(ANNOTATION_INSTANCE.filename).parentPath().string();

        ANNOTATION_INSTANCE.image = readRGBDImage(base_path + "/" + rgbd_filename);

        if (!ANNOTATION_INSTANCE.image)
        {
            QMessageBox mbox;
            mbox.setText(QString::fromStdString("Could not open '" + filename + "'."));
            mbox.exec();
            return false;
        }
    }
    else
    {
        QMessageBox mbox;
        mbox.setText(QString::fromStdString("Invalid file: " + ANNOTATION_INSTANCE.filename + ". Should contain field 'rgbd_filename'."));
        mbox.exec();
        return false;
    }

    //! 4) Read annotations
    if (r.readArray("annotations"))
    {
        while(r.nextArrayItem())
        {
            ANNOTATION_INSTANCE.annotations.push_back(Annotation());
            Annotation& a = ANNOTATION_INSTANCE.annotations.back();
            r.value("label", a.label);
            r.value("px", a.px);
            r.value("py", a.py);
        }

        r.endArray();
    }

    if (r.hasError())
    {
        std::cout << "Error while reading file '" << filename << "':\n\n" << r.error() << std::endl;
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------------------------

char show()
{
    redraw();
    cv::setMouseCallback(WINDOW_NAME, CallBackFunc, NULL);
    return cv::waitKey();
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

    tue::filesystem::Crawler crawler(path);

    tue::filesystem::Path filename;
    std::vector<std::string> filenames;
    while(crawler.nextPath(filename))
        if (filename.extension() == ".json")
            filenames.push_back(filename.string());

    TOTAL = filenames.size();
    if (TOTAL == 0)
    {
        QMessageBox mbox;
        mbox.setText(QString::fromStdString("No .json files found in directory " + path));
        mbox.exec();
        return 0;
    }
    std::cout << " == Annotation tool == \n\n Usage:\n - Click to annotate\n - Click on an annotation to modify or delete the annotation\n - Use the left/right arrow keys to browse\n - Escape to exit" << std::endl;

    //        QMessageBox intro;
    //        intro.setText(" == Annotation tool == \n\n Usage:\n - Click to annotate\n - Click on an annotation to modify or delete the annotation\n - Use the left/right arrow keys to browse\n - Escape to exit");
    //        intro.exec();

    // Load first valid file

    while(!load(filenames[INDEX]))
    {
        ++INDEX;
        if (INDEX >= TOTAL)
        {
            QMessageBox mbox;
            mbox.setText(QString::fromStdString("No valid meta-data files found in directory " + path));
            mbox.exec();
            return 0;
        }
    }

    for (;;)
    {
        char key = show();

        if (key == 81) // left
        {
            if (INDEX > 0)
            {
                int new_index = INDEX - 1;
                while(true)
                {
                    if (load(filenames[new_index]))
                    {
                        INDEX = new_index;
                        break;
                    }

                    --new_index;
                    if (new_index < 0)
                    {
                        load(filenames[INDEX]);
                        break;
                    }
                }
            }
        }
        else if (key == 83) // right
        {
            if (INDEX + 1 < TOTAL)
            {
                int new_index = INDEX + 1;
                while(true)
                {
                    if (load(filenames[new_index]))
                    {
                        INDEX = new_index;
                        break;
                    }

                    ++new_index;
                    if (new_index >= TOTAL)
                    {
                        load(filenames[INDEX]);
                        break;
                    }
                }
            }
        }
        else if (key == 27 || key == -1 || key == 'q')  // escape
        {
            break;
        }
    }

    return 0;
}

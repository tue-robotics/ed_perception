#include <QApplication>
#include <QInputDialog>
#include <QMessageBox>

#include <tue/filesystem/crawler.h>

#include <ed/uuid.h>

#include <ed/io/json_reader.h>
#include <ed/io/json_writer.h>
#include <rgbd/serialization.h>
#include <rgbd/Image.h>

#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>

struct Annotation {
    Annotation(std::string label, double px, double py) : label(label), px(px), py(py) {}

    std::string label;
    double px;
    double py;
};

struct {
    std::string filename;
    std::string json;
    rgbd::ImagePtr image;
    std::vector<Annotation> annotations;

    void clear()
    {
        filename = "";
        json = "";
        image = rgbd::ImagePtr();
        annotations.clear();
    }
} ANNOTATION_INSTANCE;

// ---------------------------------------------------------------------------------------------

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    double px = (double)x / ANNOTATION_INSTANCE.image->getRGBImage().cols;
    double py = (double)y / ANNOTATION_INSTANCE.image->getRGBImage().rows;;

    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        QInputDialog dialog;
        dialog.setLabelText("Label:");
        if (dialog.exec() && !dialog.textValue().toStdString().empty())
        {
            ANNOTATION_INSTANCE.annotations.push_back(Annotation(dialog.textValue().toStdString(), px, py));

            cv::Mat draw_img = ANNOTATION_INSTANCE.image->getRGBImage();
            cv::circle(draw_img, cv::Point2i(x,y), 6, cv::Scalar(255,0,0), 20);
            cv::putText(draw_img, dialog.textValue().toStdString().c_str(), cv::Point2i(x+15,y), 0, 1.5, cv::Scalar(255,0,255), 3);
            cv::imshow("annotation_image", draw_img);
        }
    }
    else if  ( event == cv::EVENT_RBUTTONDOWN )
    {
        //          std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if  ( event == cv::EVENT_MBUTTONDOWN )
    {
        //          std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if ( event == cv::EVENT_MOUSEMOVE )
    {
//        mouse_pos = cv::Vec2i(x, y);
    }
}

// ---------------------------------------------------------------------------------------------

rgbd::ImagePtr readRGBDImage(const std::string& filename)
{
    rgbd::ImagePtr image(new rgbd::Image);

    std::ifstream f_in;
    f_in.open(filename.c_str(), std::ifstream::binary);

    if (!f_in.is_open())
    {
        std::cout << "Could not open '" << filename << "'." << std::endl;
        return rgbd::ImagePtr();
    }

    tue::serialization::InputArchive a_in(f_in);
    rgbd::deserialize(a_in, *image);

    return image;
}

// ---------------------------------------------------------------------------------------------

bool writeMeasurement()
{
    ANNOTATION_INSTANCE.json[ANNOTATION_INSTANCE.json.size() - 1] = ',';

    std::ofstream f_out;
    f_out.open(ANNOTATION_INSTANCE.filename.c_str());
    f_out << ANNOTATION_INSTANCE.json;

    std::stringstream ss;

    ed::io::JSONWriter w(ss);
    w.writeArray("annotations");
    for (std::vector<Annotation>::const_iterator it = ANNOTATION_INSTANCE.annotations.begin(); it != ANNOTATION_INSTANCE.annotations.end(); ++it)
    {
        w.addArrayItem();
        const Annotation& a = *it;
        w.writeValue("label", a.label);
        w.writeValue("px", a.px);
        w.writeValue("py", a.py);
        w.endArrayItem();
    }
    w.endArray();

    w.finish();

    f_out << ss.str().substr(1, ss.str().size() - 2);

    f_out << "}";

    // Cleanup the ANNOTATION INSTANCE
    ANNOTATION_INSTANCE.clear();
}

// ---------------------------------------------------------------------------------------------

bool readMeasurement()
{
    std::ifstream f_in;
    f_in.open(ANNOTATION_INSTANCE.filename.c_str());

    if (!f_in.is_open())
    {
        std::cout << "Could not open '" << ANNOTATION_INSTANCE.filename << "'." << std::endl;
        return false;
    }

    std::stringstream buffer;
    buffer << f_in.rdbuf();
    ANNOTATION_INSTANCE.json = buffer.str();

    ed::io::JSONReader r(ANNOTATION_INSTANCE.json.c_str());

    if (r.readGroup("rgbd_measurement"))
    {
        std::string rgbd_filename, mask_filename;
        if (r.readValue("image_file", rgbd_filename) && r.readValue("mask_file", mask_filename))
        {
            std::string base_path = tue::filesystem::Path(ANNOTATION_INSTANCE.filename).parentPath().string();

            ANNOTATION_INSTANCE.image = readRGBDImage(base_path + "/" + rgbd_filename);

            cv::imshow("annotation_image", ANNOTATION_INSTANCE.image->getRGBImage());
            cv::setMouseCallback("annotation_image", CallBackFunc, NULL);
            cv::waitKey();

            writeMeasurement();
        }

        r.endGroup();
    }
}

// ---------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    tue::filesystem::Crawler crawler("/home/amigo/object_learning/checked");

    QApplication app(argc, argv);

    int n_measurements = 0;
    tue::filesystem::Path filename;
    while(crawler.nextPath(filename))
    {
        if (filename.extension() != ".json")
            continue;

        ANNOTATION_INSTANCE.filename = filename.string();

        readMeasurement();
    }

    return 0;
}

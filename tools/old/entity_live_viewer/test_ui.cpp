/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: May 2015
*/

// QT
#include <QApplication>

// QT gui class
#include "viewer_ui.h"

// ED
#include <ed/io/transport/probe_client.h>

// Entity Viewer Common
#include "entity_viewer_common.h"

// ROS
#include <ros/init.h>

// ----------------------------------------------------------------------------------------------------

void parseResponse(  std::vector<viewer_common::EntityInfo>& entity_list,
                     tue::serialization::InputArchive& req,
                     tue::serialization::OutputArchive& res){


}

int main( int argc, char* argv[] )
{
    std::vector<viewer_common::EntityInfo> entity_list;

    ros::init(argc, argv, "test_ui");

    ed::ProbeClient client;
    client.launchProbe("EntityViewerProbe", "libentity_viewer_probe.so");
    std::cout << "Probe launched '" << client.probeName() <<"'" << std::endl;

    QApplication app(argc, argv);

    // create viewer
    ViewerUI testUI;

    testUI.setlabeltext("");

    tue::serialization::Archive req;
    tue::serialization::Archive res;

    req << viewer_common::GET_ENTITY_LIST;

    if (client.process(req, res)){
        int num_entities;

        res >> num_entities;
        std::cout << "got " << num_entities << " entities" << std::endl;

        for(int i=0 ; i<num_entities; i++){
            std::string id;
            std::string model_name;
            std::string data;

            res >> id;
            res >> model_name;
            res >> data;

            entity_list.push_back(viewer_common::EntityInfo(id, model_name, data));
        }

        testUI.updateModelList(entity_list);

    } else {
        std::cout << "Probe processing failed!" << std::endl;
    }

    testUI.show();
    return app.exec();
}

/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: May 2015
*/

#include "viewer_ui.h"
#include "ui_entity_live_viewer_form.h"

#include <iostream>


ViewerUI::ViewerUI(QWidget *parent) :
    QMainWindow(parent),
    ui_(new Ui::MainWindow){

    ui_->setupUi(this);

    // setup entity list
    QStringList table_header;
    table_header<<"ID"<<"Model Name";
    ui_->entityTable->setColumnCount(2);
    ui_->entityTable->setHorizontalHeaderLabels(table_header);

}

ViewerUI::~ViewerUI(){
    delete ui_;
}


void ViewerUI::setlabeltext(std::string text){
    ui_->modelNameInput->setText("bananaaaaa");
}

void ViewerUI::updateModelList(std::vector<viewer_common::EntityInfo> entity_list){

//    ui_->entityTable->clear();
    int row = 0;

    for(std::vector<viewer_common::EntityInfo>::const_iterator entity_it = entity_list.begin(); entity_it != entity_list.end(); ++entity_it){
        QTableWidgetItem* item_id = new QTableWidgetItem();
        item_id->setText(entity_it->id.c_str());
        ui_->entityTable->setItem(row, 0, item_id);

//        QTableWidgetItem* item_model = new QTableWidgetItem();
//        entity_it->model_name.c_str();
//        ui_->entityTable->setItem(row, 1, item_model);

        row++;
        std::cout << "Added item: " << entity_it->id << ", " << entity_it->model_name << std::endl;
    }
}

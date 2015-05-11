
#include "viewer_ui.h"

// Qt GUI
#include <QtGui>



ViewerUI::ViewerUI(QMainWindow *parent) : QMainWindow(parent), ui_(new Ui::MainWindow){

      ui_->setupUi(this);
}

ViewerUI::~ViewerUI()
{
    delete ui_;
}


void ViewerUI::setlabeltext(std::string text){
    ui_->modelNameInput->setText("bananaaaaa");
}

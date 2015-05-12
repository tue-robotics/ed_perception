#ifndef ENTITY_LIVE_VIEWER_GUI_H
#define ENTITY_LIVE_VIEWER_GUI_H

#include "ui_entity_live_viewer_form.h"

#include <QMainWindow>
#include <QWidget>


//namespace Ui {
//    class ViewerUI;
//}

class ViewerUI : public QMainWindow{
//    Q_OBJECT

    public:
        ViewerUI(QMainWindow *parent = 0);
        ~ViewerUI();

        void setlabeltext(std::string text);

    private:
        Ui::MainWindow* ui_;
};

#endif 



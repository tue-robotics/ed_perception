#ifndef ENTITY_LIVE_VIEWER_GUI_H
#define ENTITY_LIVE_VIEWER_GUI_H

// Entity Viewer Common
#include "entity_viewer_common.h"

// QT
#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class ViewerUI : public QMainWindow{
    Q_OBJECT

    public:
        explicit ViewerUI(QWidget *parent = 0);
        ~ViewerUI();

        void setlabeltext(std::string text);
        void updateModelList(std::vector<viewer_common::EntityInfo> entity_list);

    private:
        Ui::MainWindow* ui_;

};

#endif
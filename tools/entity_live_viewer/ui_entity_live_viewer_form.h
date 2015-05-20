/********************************************************************************
** Form generated from reading UI file 'entity_live_viewer_formll7323.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef ENTITY_LIVE_VIEWER_FORMLL7323_H
#define ENTITY_LIVE_VIEWER_FORMLL7323_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGraphicsView>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTableWidget>
#include <QtGui/QTreeWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QWidget *EntityPreview;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QGraphicsView *entityPreviewView;
    QWidget *widget;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_2;
    QTreeWidget *dataTree;
    QWidget *widget_2;
    QGridLayout *gridLayout_2;
    QLabel *label_3;
    QLineEdit *modelNameInput;
    QPushButton *saveButton;
    QLabel *label_4;
    QTableWidget *entityTable;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuViewer;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(729, 771);
        MainWindow->setMinimumSize(QSize(640, 480));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        EntityPreview = new QWidget(centralwidget);
        EntityPreview->setObjectName(QString::fromUtf8("EntityPreview"));
        EntityPreview->setAutoFillBackground(false);
        verticalLayout = new QVBoxLayout(EntityPreview);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(EntityPreview);
        label->setObjectName(QString::fromUtf8("label"));
        label->setMinimumSize(QSize(101, 17));
        label->setMaximumSize(QSize(0, 0));

        verticalLayout->addWidget(label);

        entityPreviewView = new QGraphicsView(EntityPreview);
        entityPreviewView->setObjectName(QString::fromUtf8("entityPreviewView"));
        entityPreviewView->setMinimumSize(QSize(330, 309));

        verticalLayout->addWidget(entityPreviewView);


        gridLayout->addWidget(EntityPreview, 0, 0, 1, 1);

        widget = new QWidget(centralwidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        verticalLayout_2 = new QVBoxLayout(widget);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_2 = new QLabel(widget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_2->addWidget(label_2);

        dataTree = new QTreeWidget(widget);
        QTreeWidgetItem *__qtreewidgetitem = new QTreeWidgetItem();
        __qtreewidgetitem->setText(0, QString::fromUtf8("Data"));
        dataTree->setHeaderItem(__qtreewidgetitem);
        dataTree->setObjectName(QString::fromUtf8("dataTree"));
        dataTree->setMinimumSize(QSize(291, 531));

        verticalLayout_2->addWidget(dataTree);


        gridLayout->addWidget(widget, 0, 1, 2, 1);

        widget_2 = new QWidget(centralwidget);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        gridLayout_2 = new QGridLayout(widget_2);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label_3 = new QLabel(widget_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setMinimumSize(QSize(91, 17));
        label_3->setMaximumSize(QSize(91, 17));

        gridLayout_2->addWidget(label_3, 0, 0, 1, 1);

        modelNameInput = new QLineEdit(widget_2);
        modelNameInput->setObjectName(QString::fromUtf8("modelNameInput"));
        modelNameInput->setMinimumSize(QSize(183, 27));

        gridLayout_2->addWidget(modelNameInput, 1, 0, 1, 1);

        saveButton = new QPushButton(widget_2);
        saveButton->setObjectName(QString::fromUtf8("saveButton"));
        saveButton->setMinimumSize(QSize(141, 27));
        saveButton->setMaximumSize(QSize(141, 27));

        gridLayout_2->addWidget(saveButton, 1, 1, 1, 1);

        label_4 = new QLabel(widget_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setMaximumSize(QSize(90, 17));

        gridLayout_2->addWidget(label_4, 2, 0, 1, 1);

        entityTable = new QTableWidget(widget_2);
        entityTable->setObjectName(QString::fromUtf8("entityTable"));
        entityTable->setColumnCount(0);

        gridLayout_2->addWidget(entityTable, 3, 0, 1, 2);


        gridLayout->addWidget(widget_2, 1, 0, 1, 1);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 729, 25));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuViewer = new QMenu(menubar);
        menuViewer->setObjectName(QString::fromUtf8("menuViewer"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuViewer->menuAction());

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Entity Live Viewer", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Entity Preview", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Entity Data", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Model Name", 0, QApplication::UnicodeUTF8));
        saveButton->setText(QApplication::translate("MainWindow", "Save Measurement", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Entity List", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuViewer->setTitle(QApplication::translate("MainWindow", "Viewer", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // ENTITY_LIVE_VIEWER_FORMLL7323_H

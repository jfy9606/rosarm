/********************************************************************************
** Form generated from reading UI file 'arm_control_main.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ARM_CONTROL_MAIN_H
#define UI_ARM_CONTROL_MAIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QOpenGLWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ArmControlMainWindow
{
public:
    QAction *actionOpen_Task_Sequence;
    QAction *actionSave_Task_Sequence;
    QAction *actionExit;
    QAction *actionCamera_Settings;
    QAction *actionRobot_Settings;
    QAction *actionAbout;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QSplitter *splitter;
    QWidget *controlWidget;
    QVBoxLayout *verticalLayout;
    QGroupBox *jointControlGroup;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_joint1;
    QHBoxLayout *horizontalLayout_2;
    QSlider *joint1_slider;
    QDoubleSpinBox *joint1_spin;
    QLabel *label_joint2;
    QHBoxLayout *horizontalLayout_3;
    QSlider *joint2_slider;
    QDoubleSpinBox *joint2_spin;
    QLabel *label_joint3;
    QHBoxLayout *horizontalLayout_4;
    QSlider *joint3_slider;
    QDoubleSpinBox *joint3_spin;
    QLabel *label_joint4;
    QHBoxLayout *horizontalLayout_5;
    QSlider *joint4_slider;
    QDoubleSpinBox *joint4_spin;
    QLabel *label_joint6;
    QHBoxLayout *horizontalLayout_6;
    QSlider *joint6_slider;
    QDoubleSpinBox *joint6_spin;
    QGroupBox *endEffectorGroup;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_2;
    QLabel *label_3;
    QDoubleSpinBox *pos_x;
    QLabel *label_4;
    QDoubleSpinBox *pos_y;
    QLabel *label_5;
    QDoubleSpinBox *pos_z;
    QHBoxLayout *horizontalLayout_8;
    QPushButton *moveToPositionButton;
    QPushButton *homeButton;
    QGroupBox *vacuumGroup;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label;
    QSlider *vacuumPowerSlider;
    QLabel *vacuumPowerLabel;
    QHBoxLayout *horizontalLayout_10;
    QPushButton *vacuumOnButton;
    QPushButton *vacuumOffButton;
    QGroupBox *taskGroup;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_11;
    QPushButton *pickButton;
    QPushButton *placeButton;
    QPushButton *sequenceButton;
    QTextEdit *logTextEdit;
    QTabWidget *tabWidget;
    QWidget *viewTab;
    QVBoxLayout *verticalLayout_6;
    QOpenGLWidget *armView;
    QWidget *cameraTab;
    QVBoxLayout *verticalLayout_7;
    QWidget *cameraViewContainer;
    QHBoxLayout *horizontalLayout_12;
    QLabel *cameraViewLeft;
    QLabel *cameraViewRight;
    QLabel *depthView;
    QWidget *detectionsTab;
    QVBoxLayout *verticalLayout_8;
    QLabel *detectionView;
    QTableWidget *detectionsTable;
    QMenuBar *menubar;
    QMenu *menu_File;
    QMenu *menu_Settings;
    QMenu *menu_Help;
    QStatusBar *statusbar;
    QToolBar *toolBar;

    void setupUi(QMainWindow *ArmControlMainWindow)
    {
        if (ArmControlMainWindow->objectName().isEmpty())
            ArmControlMainWindow->setObjectName(QString::fromUtf8("ArmControlMainWindow"));
        ArmControlMainWindow->resize(1200, 800);
        actionOpen_Task_Sequence = new QAction(ArmControlMainWindow);
        actionOpen_Task_Sequence->setObjectName(QString::fromUtf8("actionOpen_Task_Sequence"));
        actionSave_Task_Sequence = new QAction(ArmControlMainWindow);
        actionSave_Task_Sequence->setObjectName(QString::fromUtf8("actionSave_Task_Sequence"));
        actionExit = new QAction(ArmControlMainWindow);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionCamera_Settings = new QAction(ArmControlMainWindow);
        actionCamera_Settings->setObjectName(QString::fromUtf8("actionCamera_Settings"));
        actionRobot_Settings = new QAction(ArmControlMainWindow);
        actionRobot_Settings->setObjectName(QString::fromUtf8("actionRobot_Settings"));
        actionAbout = new QAction(ArmControlMainWindow);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        centralwidget = new QWidget(ArmControlMainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        splitter = new QSplitter(centralwidget);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        controlWidget = new QWidget(splitter);
        controlWidget->setObjectName(QString::fromUtf8("controlWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(controlWidget->sizePolicy().hasHeightForWidth());
        controlWidget->setSizePolicy(sizePolicy);
        controlWidget->setMinimumSize(QSize(350, 0));
        verticalLayout = new QVBoxLayout(controlWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        jointControlGroup = new QGroupBox(controlWidget);
        jointControlGroup->setObjectName(QString::fromUtf8("jointControlGroup"));
        verticalLayout_2 = new QVBoxLayout(jointControlGroup);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_joint1 = new QLabel(jointControlGroup);
        label_joint1->setObjectName(QString::fromUtf8("label_joint1"));

        verticalLayout_2->addWidget(label_joint1);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        joint1_slider = new QSlider(jointControlGroup);
        joint1_slider->setObjectName(QString::fromUtf8("joint1_slider"));
        joint1_slider->setMinimum(-180);
        joint1_slider->setMaximum(180);
        joint1_slider->setOrientation(Qt::Horizontal);
        joint1_slider->setTickPosition(QSlider::TicksBelow);

        horizontalLayout_2->addWidget(joint1_slider);

        joint1_spin = new QDoubleSpinBox(jointControlGroup);
        joint1_spin->setObjectName(QString::fromUtf8("joint1_spin"));
        joint1_spin->setMinimum(-180.000000000000000);
        joint1_spin->setMaximum(180.000000000000000);

        horizontalLayout_2->addWidget(joint1_spin);


        verticalLayout_2->addLayout(horizontalLayout_2);

        label_joint2 = new QLabel(jointControlGroup);
        label_joint2->setObjectName(QString::fromUtf8("label_joint2"));

        verticalLayout_2->addWidget(label_joint2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        joint2_slider = new QSlider(jointControlGroup);
        joint2_slider->setObjectName(QString::fromUtf8("joint2_slider"));
        joint2_slider->setMinimum(0);
        joint2_slider->setMaximum(43);
        joint2_slider->setOrientation(Qt::Horizontal);
        joint2_slider->setTickPosition(QSlider::TicksBelow);

        horizontalLayout_3->addWidget(joint2_slider);

        joint2_spin = new QDoubleSpinBox(jointControlGroup);
        joint2_spin->setObjectName(QString::fromUtf8("joint2_spin"));
        joint2_spin->setMinimum(0.000000000000000);
        joint2_spin->setMaximum(43.000000000000000);

        horizontalLayout_3->addWidget(joint2_spin);


        verticalLayout_2->addLayout(horizontalLayout_3);

        label_joint3 = new QLabel(jointControlGroup);
        label_joint3->setObjectName(QString::fromUtf8("label_joint3"));

        verticalLayout_2->addWidget(label_joint3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        joint3_slider = new QSlider(jointControlGroup);
        joint3_slider->setObjectName(QString::fromUtf8("joint3_slider"));
        joint3_slider->setMinimum(-90);
        joint3_slider->setMaximum(90);
        joint3_slider->setOrientation(Qt::Horizontal);
        joint3_slider->setTickPosition(QSlider::TicksBelow);

        horizontalLayout_4->addWidget(joint3_slider);

        joint3_spin = new QDoubleSpinBox(jointControlGroup);
        joint3_spin->setObjectName(QString::fromUtf8("joint3_spin"));
        joint3_spin->setMinimum(-90.000000000000000);
        joint3_spin->setMaximum(90.000000000000000);

        horizontalLayout_4->addWidget(joint3_spin);


        verticalLayout_2->addLayout(horizontalLayout_4);

        label_joint4 = new QLabel(jointControlGroup);
        label_joint4->setObjectName(QString::fromUtf8("label_joint4"));

        verticalLayout_2->addWidget(label_joint4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        joint4_slider = new QSlider(jointControlGroup);
        joint4_slider->setObjectName(QString::fromUtf8("joint4_slider"));
        joint4_slider->setMinimum(0);
        joint4_slider->setMaximum(180);
        joint4_slider->setOrientation(Qt::Horizontal);
        joint4_slider->setTickPosition(QSlider::TicksBelow);

        horizontalLayout_5->addWidget(joint4_slider);

        joint4_spin = new QDoubleSpinBox(jointControlGroup);
        joint4_spin->setObjectName(QString::fromUtf8("joint4_spin"));
        joint4_spin->setMinimum(0.000000000000000);
        joint4_spin->setMaximum(180.000000000000000);

        horizontalLayout_5->addWidget(joint4_spin);


        verticalLayout_2->addLayout(horizontalLayout_5);

        label_joint6 = new QLabel(jointControlGroup);
        label_joint6->setObjectName(QString::fromUtf8("label_joint6"));

        verticalLayout_2->addWidget(label_joint6);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        joint6_slider = new QSlider(jointControlGroup);
        joint6_slider->setObjectName(QString::fromUtf8("joint6_slider"));
        joint6_slider->setMinimum(5);
        joint6_slider->setMaximum(15);
        joint6_slider->setOrientation(Qt::Horizontal);
        joint6_slider->setTickPosition(QSlider::TicksBelow);

        horizontalLayout_6->addWidget(joint6_slider);

        joint6_spin = new QDoubleSpinBox(jointControlGroup);
        joint6_spin->setObjectName(QString::fromUtf8("joint6_spin"));
        joint6_spin->setMinimum(5.000000000000000);
        joint6_spin->setMaximum(15.000000000000000);

        horizontalLayout_6->addWidget(joint6_spin);


        verticalLayout_2->addLayout(horizontalLayout_6);


        verticalLayout->addWidget(jointControlGroup);

        endEffectorGroup = new QGroupBox(controlWidget);
        endEffectorGroup->setObjectName(QString::fromUtf8("endEffectorGroup"));
        verticalLayout_3 = new QVBoxLayout(endEffectorGroup);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        label_2 = new QLabel(endEffectorGroup);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_7->addWidget(label_2);

        label_3 = new QLabel(endEffectorGroup);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_7->addWidget(label_3);

        pos_x = new QDoubleSpinBox(endEffectorGroup);
        pos_x->setObjectName(QString::fromUtf8("pos_x"));
        pos_x->setMinimum(-100.000000000000000);
        pos_x->setMaximum(100.000000000000000);

        horizontalLayout_7->addWidget(pos_x);

        label_4 = new QLabel(endEffectorGroup);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_7->addWidget(label_4);

        pos_y = new QDoubleSpinBox(endEffectorGroup);
        pos_y->setObjectName(QString::fromUtf8("pos_y"));
        pos_y->setMinimum(-100.000000000000000);
        pos_y->setMaximum(100.000000000000000);

        horizontalLayout_7->addWidget(pos_y);

        label_5 = new QLabel(endEffectorGroup);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_7->addWidget(label_5);

        pos_z = new QDoubleSpinBox(endEffectorGroup);
        pos_z->setObjectName(QString::fromUtf8("pos_z"));
        pos_z->setMinimum(-100.000000000000000);
        pos_z->setMaximum(100.000000000000000);

        horizontalLayout_7->addWidget(pos_z);


        verticalLayout_3->addLayout(horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        moveToPositionButton = new QPushButton(endEffectorGroup);
        moveToPositionButton->setObjectName(QString::fromUtf8("moveToPositionButton"));

        horizontalLayout_8->addWidget(moveToPositionButton);

        homeButton = new QPushButton(endEffectorGroup);
        homeButton->setObjectName(QString::fromUtf8("homeButton"));

        horizontalLayout_8->addWidget(homeButton);


        verticalLayout_3->addLayout(horizontalLayout_8);


        verticalLayout->addWidget(endEffectorGroup);

        vacuumGroup = new QGroupBox(controlWidget);
        vacuumGroup->setObjectName(QString::fromUtf8("vacuumGroup"));
        verticalLayout_4 = new QVBoxLayout(vacuumGroup);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label = new QLabel(vacuumGroup);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_9->addWidget(label);

        vacuumPowerSlider = new QSlider(vacuumGroup);
        vacuumPowerSlider->setObjectName(QString::fromUtf8("vacuumPowerSlider"));
        vacuumPowerSlider->setMaximum(100);
        vacuumPowerSlider->setOrientation(Qt::Horizontal);
        vacuumPowerSlider->setTickPosition(QSlider::TicksBelow);

        horizontalLayout_9->addWidget(vacuumPowerSlider);

        vacuumPowerLabel = new QLabel(vacuumGroup);
        vacuumPowerLabel->setObjectName(QString::fromUtf8("vacuumPowerLabel"));

        horizontalLayout_9->addWidget(vacuumPowerLabel);


        verticalLayout_4->addLayout(horizontalLayout_9);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        vacuumOnButton = new QPushButton(vacuumGroup);
        vacuumOnButton->setObjectName(QString::fromUtf8("vacuumOnButton"));

        horizontalLayout_10->addWidget(vacuumOnButton);

        vacuumOffButton = new QPushButton(vacuumGroup);
        vacuumOffButton->setObjectName(QString::fromUtf8("vacuumOffButton"));

        horizontalLayout_10->addWidget(vacuumOffButton);


        verticalLayout_4->addLayout(horizontalLayout_10);


        verticalLayout->addWidget(vacuumGroup);

        taskGroup = new QGroupBox(controlWidget);
        taskGroup->setObjectName(QString::fromUtf8("taskGroup"));
        verticalLayout_5 = new QVBoxLayout(taskGroup);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        pickButton = new QPushButton(taskGroup);
        pickButton->setObjectName(QString::fromUtf8("pickButton"));

        horizontalLayout_11->addWidget(pickButton);

        placeButton = new QPushButton(taskGroup);
        placeButton->setObjectName(QString::fromUtf8("placeButton"));

        horizontalLayout_11->addWidget(placeButton);


        verticalLayout_5->addLayout(horizontalLayout_11);

        sequenceButton = new QPushButton(taskGroup);
        sequenceButton->setObjectName(QString::fromUtf8("sequenceButton"));

        verticalLayout_5->addWidget(sequenceButton);


        verticalLayout->addWidget(taskGroup);

        logTextEdit = new QTextEdit(controlWidget);
        logTextEdit->setObjectName(QString::fromUtf8("logTextEdit"));
        logTextEdit->setReadOnly(true);

        verticalLayout->addWidget(logTextEdit);

        splitter->addWidget(controlWidget);
        tabWidget = new QTabWidget(splitter);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(2);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy1);
        viewTab = new QWidget();
        viewTab->setObjectName(QString::fromUtf8("viewTab"));
        verticalLayout_6 = new QVBoxLayout(viewTab);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        armView = new QOpenGLWidget(viewTab);
        armView->setObjectName(QString::fromUtf8("armView"));

        verticalLayout_6->addWidget(armView);

        tabWidget->addTab(viewTab, QString());
        cameraTab = new QWidget();
        cameraTab->setObjectName(QString::fromUtf8("cameraTab"));
        verticalLayout_7 = new QVBoxLayout(cameraTab);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        cameraViewContainer = new QWidget(cameraTab);
        cameraViewContainer->setObjectName(QString::fromUtf8("cameraViewContainer"));
        horizontalLayout_12 = new QHBoxLayout(cameraViewContainer);
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        cameraViewLeft = new QLabel(cameraViewContainer);
        cameraViewLeft->setObjectName(QString::fromUtf8("cameraViewLeft"));
        cameraViewLeft->setMinimumSize(QSize(320, 240));
        cameraViewLeft->setFrameShape(QFrame::Box);
        cameraViewLeft->setAlignment(Qt::AlignCenter);

        horizontalLayout_12->addWidget(cameraViewLeft);

        cameraViewRight = new QLabel(cameraViewContainer);
        cameraViewRight->setObjectName(QString::fromUtf8("cameraViewRight"));
        cameraViewRight->setMinimumSize(QSize(320, 240));
        cameraViewRight->setFrameShape(QFrame::Box);
        cameraViewRight->setAlignment(Qt::AlignCenter);

        horizontalLayout_12->addWidget(cameraViewRight);


        verticalLayout_7->addWidget(cameraViewContainer);

        depthView = new QLabel(cameraTab);
        depthView->setObjectName(QString::fromUtf8("depthView"));
        depthView->setMinimumSize(QSize(0, 240));
        depthView->setFrameShape(QFrame::Box);
        depthView->setAlignment(Qt::AlignCenter);

        verticalLayout_7->addWidget(depthView);

        tabWidget->addTab(cameraTab, QString());
        detectionsTab = new QWidget();
        detectionsTab->setObjectName(QString::fromUtf8("detectionsTab"));
        verticalLayout_8 = new QVBoxLayout(detectionsTab);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        detectionView = new QLabel(detectionsTab);
        detectionView->setObjectName(QString::fromUtf8("detectionView"));
        detectionView->setMinimumSize(QSize(640, 480));
        detectionView->setFrameShape(QFrame::Box);
        detectionView->setAlignment(Qt::AlignCenter);

        verticalLayout_8->addWidget(detectionView);

        detectionsTable = new QTableWidget(detectionsTab);
        if (detectionsTable->columnCount() < 6)
            detectionsTable->setColumnCount(6);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        detectionsTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        detectionsTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        detectionsTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        detectionsTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        detectionsTable->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        detectionsTable->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        detectionsTable->setObjectName(QString::fromUtf8("detectionsTable"));
        detectionsTable->setColumnCount(6);

        verticalLayout_8->addWidget(detectionsTable);

        tabWidget->addTab(detectionsTab, QString());
        splitter->addWidget(tabWidget);

        horizontalLayout->addWidget(splitter);

        ArmControlMainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(ArmControlMainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1200, 22));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        menu_Settings = new QMenu(menubar);
        menu_Settings->setObjectName(QString::fromUtf8("menu_Settings"));
        menu_Help = new QMenu(menubar);
        menu_Help->setObjectName(QString::fromUtf8("menu_Help"));
        ArmControlMainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(ArmControlMainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        ArmControlMainWindow->setStatusBar(statusbar);
        toolBar = new QToolBar(ArmControlMainWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        ArmControlMainWindow->addToolBar(Qt::TopToolBarArea, toolBar);

        menubar->addAction(menu_File->menuAction());
        menubar->addAction(menu_Settings->menuAction());
        menubar->addAction(menu_Help->menuAction());
        menu_File->addAction(actionOpen_Task_Sequence);
        menu_File->addAction(actionSave_Task_Sequence);
        menu_File->addAction(actionExit);
        menu_Settings->addAction(actionCamera_Settings);
        menu_Settings->addAction(actionRobot_Settings);
        menu_Help->addAction(actionAbout);
        toolBar->addAction(actionOpen_Task_Sequence);
        toolBar->addAction(actionSave_Task_Sequence);
        toolBar->addSeparator();
        toolBar->addAction(actionCamera_Settings);

        retranslateUi(ArmControlMainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(ArmControlMainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *ArmControlMainWindow)
    {
        ArmControlMainWindow->setWindowTitle(QApplication::translate("ArmControlMainWindow", "\346\234\272\346\242\260\350\207\202\346\216\247\345\210\266\351\235\242\346\235\277", nullptr));
        actionOpen_Task_Sequence->setText(QApplication::translate("ArmControlMainWindow", "\346\211\223\345\274\200\344\273\273\345\212\241\345\272\217\345\210\227", nullptr));
        actionSave_Task_Sequence->setText(QApplication::translate("ArmControlMainWindow", "\344\277\235\345\255\230\344\273\273\345\212\241\345\272\217\345\210\227", nullptr));
        actionExit->setText(QApplication::translate("ArmControlMainWindow", "\351\200\200\345\207\272", nullptr));
        actionCamera_Settings->setText(QApplication::translate("ArmControlMainWindow", "\346\221\204\345\203\217\345\244\264\350\256\276\347\275\256", nullptr));
        actionRobot_Settings->setText(QApplication::translate("ArmControlMainWindow", "\346\234\272\346\242\260\350\207\202\350\256\276\347\275\256", nullptr));
        actionAbout->setText(QApplication::translate("ArmControlMainWindow", "\345\205\263\344\272\216", nullptr));
        jointControlGroup->setTitle(QApplication::translate("ArmControlMainWindow", "\345\205\263\350\212\202\346\216\247\345\210\266", nullptr));
        label_joint1->setText(QApplication::translate("ArmControlMainWindow", "\345\272\225\345\272\247\346\227\213\350\275\254 (\316\2701)\357\274\232\302\261180\302\260", nullptr));
        label_joint2->setText(QApplication::translate("ArmControlMainWindow", "\344\274\270\347\274\251\345\205\263\350\212\202 (d2)\357\274\2320-43cm", nullptr));
        label_joint3->setText(QApplication::translate("ArmControlMainWindow", "\350\202\251\351\203\250\345\205\263\350\212\202 (\316\2703)\357\274\232\302\26190\302\260", nullptr));
        label_joint4->setText(QApplication::translate("ArmControlMainWindow", "\350\202\230\351\203\250\345\205\263\350\212\202 (\316\2704)\357\274\2320-180\302\260", nullptr));
        label_joint6->setText(QApplication::translate("ArmControlMainWindow", "\346\234\253\347\253\257\344\274\270\347\274\251 (d6)\357\274\2325-15cm", nullptr));
        endEffectorGroup->setTitle(QApplication::translate("ArmControlMainWindow", "\346\234\253\347\253\257\346\211\247\350\241\214\345\231\250\346\216\247\345\210\266", nullptr));
        label_2->setText(QApplication::translate("ArmControlMainWindow", "\344\275\215\347\275\256\357\274\232", nullptr));
        label_3->setText(QApplication::translate("ArmControlMainWindow", "X:", nullptr));
        label_4->setText(QApplication::translate("ArmControlMainWindow", "Y:", nullptr));
        label_5->setText(QApplication::translate("ArmControlMainWindow", "Z:", nullptr));
        moveToPositionButton->setText(QApplication::translate("ArmControlMainWindow", "\347\247\273\345\212\250\345\210\260\346\214\207\345\256\232\344\275\215\347\275\256", nullptr));
        homeButton->setText(QApplication::translate("ArmControlMainWindow", "\345\210\235\345\247\213\344\275\215\347\275\256", nullptr));
        vacuumGroup->setTitle(QApplication::translate("ArmControlMainWindow", "\345\220\270\351\231\204\346\216\247\345\210\266", nullptr));
        label->setText(QApplication::translate("ArmControlMainWindow", "\351\243\216\346\234\272\345\212\237\347\216\207:", nullptr));
        vacuumPowerLabel->setText(QApplication::translate("ArmControlMainWindow", "0%", nullptr));
        vacuumOnButton->setText(QApplication::translate("ArmControlMainWindow", "\345\274\200\345\220\257\345\220\270\351\231\204", nullptr));
        vacuumOffButton->setText(QApplication::translate("ArmControlMainWindow", "\345\205\263\351\227\255\345\220\270\351\231\204", nullptr));
        taskGroup->setTitle(QApplication::translate("ArmControlMainWindow", "\344\273\273\345\212\241\346\216\247\345\210\266", nullptr));
        pickButton->setText(QApplication::translate("ArmControlMainWindow", "\346\212\223\345\217\226\347\211\251\344\275\223", nullptr));
        placeButton->setText(QApplication::translate("ArmControlMainWindow", "\346\224\276\347\275\256\347\211\251\344\275\223", nullptr));
        sequenceButton->setText(QApplication::translate("ArmControlMainWindow", "\346\211\247\350\241\214\351\242\204\350\256\276\344\273\273\345\212\241\345\272\217\345\210\227", nullptr));
        logTextEdit->setPlaceholderText(QApplication::translate("ArmControlMainWindow", "\346\227\245\345\277\227\344\277\241\346\201\257...", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(viewTab), QApplication::translate("ArmControlMainWindow", "3D\350\247\206\345\233\276", nullptr));
        cameraViewLeft->setText(QApplication::translate("ArmControlMainWindow", "\345\267\246\346\221\204\345\203\217\345\244\264\350\247\206\345\233\276", nullptr));
        cameraViewRight->setText(QApplication::translate("ArmControlMainWindow", "\345\217\263\346\221\204\345\203\217\345\244\264\350\247\206\345\233\276", nullptr));
        depthView->setText(QApplication::translate("ArmControlMainWindow", "\346\267\261\345\272\246\345\233\276", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(cameraTab), QApplication::translate("ArmControlMainWindow", "\346\221\204\345\203\217\345\244\264\350\247\206\345\233\276", nullptr));
        detectionView->setText(QApplication::translate("ArmControlMainWindow", "\347\211\251\344\275\223\346\243\200\346\265\213\350\247\206\345\233\276", nullptr));
        QTableWidgetItem *___qtablewidgetitem = detectionsTable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("ArmControlMainWindow", "ID", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = detectionsTable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("ArmControlMainWindow", "\347\261\273\345\236\213", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = detectionsTable->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QApplication::translate("ArmControlMainWindow", "X (cm)", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = detectionsTable->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QApplication::translate("ArmControlMainWindow", "Y (cm)", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = detectionsTable->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QApplication::translate("ArmControlMainWindow", "Z (cm)", nullptr));
        QTableWidgetItem *___qtablewidgetitem5 = detectionsTable->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QApplication::translate("ArmControlMainWindow", "\346\223\215\344\275\234", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(detectionsTab), QApplication::translate("ArmControlMainWindow", "\347\211\251\344\275\223\346\243\200\346\265\213", nullptr));
        menu_File->setTitle(QApplication::translate("ArmControlMainWindow", "\346\226\207\344\273\266", nullptr));
        menu_Settings->setTitle(QApplication::translate("ArmControlMainWindow", "\350\256\276\347\275\256", nullptr));
        menu_Help->setTitle(QApplication::translate("ArmControlMainWindow", "\345\270\256\345\212\251", nullptr));
        toolBar->setWindowTitle(QApplication::translate("ArmControlMainWindow", "toolBar", nullptr));
    } // retranslateUi

};

namespace Ui {
    class ArmControlMainWindow: public Ui_ArmControlMainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ARM_CONTROL_MAIN_H

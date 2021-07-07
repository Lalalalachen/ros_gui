/**
 * @file /include/ros_qt_demo/main_window.hpp
 *
 * @brief Qt based gui for ros_qt_demo.
 *
 * @date November 2010
 **/
#ifndef ros_qt_demo_MAIN_WINDOW_H
#define ros_qt_demo_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QProcess>
#include <QComboBox>
#include "qrviz.hpp"
#include <QSpinBox>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ros_qt_demo {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void slot_update_image(QImage);
    void slot_sub_image();
    void slot_quick_cmd_clicked();
    void slot_roscore_cmd_clicked();
    void slot_exit_roscore_clicked();
    void slot_roscore_output();
    void slot_treewidget_value_change(QString);
    void slot_display_grid(int);
    void slot_display_tf(int);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  QProcess *roscore_cmd;
  QProcess *roscore_exit_cmd;
  qrviz* myrviz;
  QComboBox* fixed_box;
  QSpinBox* Cell_Count_Box;
  QComboBox* Grid_Color_Box;
  qrviz* front_view;
  qrviz* side_view;
  qrviz* top_view;
};

}  // namespace ros_qt_demo

#endif // ros_qt_demo_MAIN_WINDOW_H

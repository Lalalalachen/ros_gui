/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/ros_qt_demo/main_window.hpp"
#include <QProcess>
//#include "../include/ros_qt_demo/qrviz.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_qt_demo {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    //ui init
    ui.treeWidget->setWindowTitle("Display");
    ui.treeWidget->setHeaderLabels(QStringList() << "key" << "value");
    ui.treeWidget->setHeaderHidden(true);

    //Global Options
    QTreeWidgetItem* Global = new QTreeWidgetItem(QStringList() << "Global Options");
    ui.treeWidget->addTopLevelItem(Global);
    Global->setExpanded(true);

    //fix frame
    QTreeWidgetItem* Fixed_frame = new QTreeWidgetItem(QStringList() << "Fixed Frame");
    fixed_box = new QComboBox();
    fixed_box->addItem("map");
    fixed_box->setMaximumWidth(150);
    fixed_box->setEditable(true);

    Global->addChild(Fixed_frame);
    connect(fixed_box, SIGNAL(currentTextChanged(QString)),
            this, SLOT(slot_treewidget_value_change(QString)));
    ui.treeWidget->setItemWidget(Fixed_frame, 1, fixed_box);

    //Grid
    QTreeWidgetItem* Grid = new QTreeWidgetItem(QStringList() << "Grid");
    //checbox
    QCheckBox* Grid_Check = new QCheckBox();

    //cannot default checked
    //Grid_Check->setCheckState(Qt::Checked);
    //connect SLOT to display
    connect(Grid_Check, SIGNAL(stateChanged(int)), this, SLOT(slot_display_grid(int)));

    //add top item
    ui.treeWidget->addTopLevelItem(Grid);
    //add checkbox
    ui.treeWidget->setItemWidget(Grid, 1, Grid_Check);
    //default expanded
    Grid->setExpanded(true);

    //add CellCount and Color sub-items
    QTreeWidgetItem* Cell_Count = new QTreeWidgetItem(QStringList() << "Plan Cell Count");
    Grid->addChild(Cell_Count);
    Cell_Count_Box = new QSpinBox();
    Cell_Count_Box->setValue(13);
    Cell_Count_Box->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(Cell_Count, 1, Cell_Count_Box);

    QTreeWidgetItem* Grid_Color = new QTreeWidgetItem(QStringList() << "Color");
    Grid->addChild(Grid_Color);
    Grid_Color_Box = new QComboBox();
    Grid_Color_Box->addItem("160;160;160");
    Grid_Color_Box->setEditable(true);
    Grid_Color_Box->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(Grid_Color, 1, Grid_Color_Box);

    //TF
    QTreeWidgetItem* TF = new QTreeWidgetItem(QStringList() << "TF");
    //checkbox
    QCheckBox* TF_Check = new QCheckBox();
    connect(TF_Check, SIGNAL(stateChanged(int)), this, SLOT(slot_display_tf(int)));

    ui.treeWidget->addTopLevelItem(TF);
    ui.treeWidget->setItemWidget(TF, 1, TF_Check);


    //connect
    //image
    connect(&qnode, SIGNAL(image_vel(QImage)), this, SLOT(slot_update_image(QImage)));
    connect(ui.pushButton_sub_image, SIGNAL(clicked()), this, SLOT(slot_sub_image()));
    //quick command
    connect(ui.pushButton_cmd, SIGNAL(clicked()), this, SLOT(slot_quick_cmd_clicked()));
    //roscore command
    connect(ui.pushButton_roscore, SIGNAL(clicked()), this, SLOT(slot_roscore_cmd_clicked()));
    connect(ui.pushButton_exit_roscore, SIGNAL(clicked()), this, SLOT(slot_exit_roscore_clicked()));
    //connect(ui.quit_button, SIGNAL(clicked()), this, SLOT(slot_exit_roscore_clicked()));

    //bag file path
    connect(ui.pushButton_path, SIGNAL(clicked()), this, SLOT(slot_file_path()));

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::slot_file_path() {
  QString bag_file, bag_file_path;
  bag_file = QFileDialog::getOpenFileName(this,
                                          "Please choose a bag file",
                                          "/",
                                          "Bag Files(*.bag);;All(*.*");
  if(bag_file != "") {
    QFileInfo bag_file_info;
    bag_file_info = QFileInfo(bag_file);
    bag_file_path = bag_file_info.filePath();
    ui.lineEdit_path->setText(bag_file_path);
    // ros-launch nodes and play bag file
    quick_cmd_bag = new QProcess();
    quick_cmd_bag->start("bash");
    quick_cmd_bag->write("rosbag play ");
    quick_cmd_bag->write(bag_file_path.toLocal8Bit());
    //quick_cmd_bag->write("/mnt/hgfs/VM-Ubuntu/Calibration/record.bag");
    quick_cmd_bag->write(" -l\n");

    quick_cmd_launch = new QProcess();
    quick_cmd_launch->start("bash");
    quick_cmd_launch->write("roslaunch temporary_calib_interface calib_vis_bag.launch\n");
  }

}

void MainWindow::slot_display_tf(int state) {
  bool enable = state>1? true:false;
  myrviz->Display_TF(enable);
  front_view->Display_TF(enable);
  side_view->Display_TF(enable);
  top_view->Display_TF(enable);
}

void MainWindow::slot_display_grid(int state) {
  bool enable = state>1? true:false;
  QStringList qsl = Grid_Color_Box->currentText().split(";");
  QColor color = QColor(qsl[0].toInt(), qsl[1].toInt(), qsl[2].toInt());
  myrviz->Display_Grid(Cell_Count_Box->text().toInt(), color, enable);
  front_view->Display_Grid(Cell_Count_Box->text().toInt(), color, enable);
  side_view->Display_Grid(Cell_Count_Box->text().toInt(), color, enable);
  top_view->Display_Grid(Cell_Count_Box->text().toInt(), color, enable);
}

void MainWindow::slot_treewidget_value_change(QString) {
  myrviz->Set_FixedFrame(fixed_box->currentText());
  front_view->Set_FixedFrame(fixed_box->currentText());
  side_view->Set_FixedFrame(fixed_box->currentText());
  top_view->Set_FixedFrame(fixed_box->currentText());
}

void MainWindow::slot_roscore_output() {
  ui.textEdit_roscore_output->append(roscore_cmd->readAllStandardOutput().data());
  ui.textEdit_roscore_output->append(roscore_cmd->readAllStandardError().data());
}

void MainWindow::slot_exit_roscore_clicked() {
  roscore_exit_cmd = new QProcess();
  roscore_exit_cmd->start("bash");
  roscore_exit_cmd->waitForStarted();
  roscore_exit_cmd->write("killall -9 rosmaster\n");
}

void MainWindow::slot_roscore_cmd_clicked() {
  roscore_cmd = new QProcess();
  roscore_cmd->start("bash");
  roscore_cmd->waitForStarted();
  roscore_cmd->write("roscore\n");
  connect(roscore_cmd, SIGNAL(readyReadStandardOutput()),
          this, SLOT(slot_roscore_output()));
  connect(roscore_cmd, SIGNAL(readyReadStandardError()),
          this, SLOT(slot_roscore_output()));
}

void MainWindow::slot_quick_cmd_clicked() {
  quick_cmd = new QProcess();
  quick_cmd->start("bash");
  quick_cmd->write(ui.textEdit_cmd->toPlainText().toLocal8Bit()+'\n');
}
void MainWindow::slot_update_image(QImage im) {
  QSize laSize = ui.label_image->size();
  QImage image_scaled = im.scaled(laSize, Qt::IgnoreAspectRatio);
  ui.label_image->setPixmap(QPixmap::fromImage(image_scaled));
}

void MainWindow::slot_sub_image() {
  qnode.sub_image(ui.lineEdit_image_topic->text());
}

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
      // if cannot connect master, cannot use Display
      ui.treeWidget->setEnabled(false);
		} else {
			ui.button_connect->setEnabled(false);

      ui.treeWidget->setEnabled(true);
      // init qrviz class
      myrviz = new qrviz(ui.layout_rviz);
      // init qrviz class for three view
      front_view = new qrviz(ui.layout_front);
      side_view = new qrviz(ui.layout_side);
      top_view = new qrviz(ui.layout_top);


		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
      ui.treeWidget->setEnabled(false);
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);

      ui.treeWidget->setEnabled(true);
      // init qrviz class
      myrviz = new qrviz(ui.layout_rviz);

      // init qrviz class for three view
      front_view = new qrviz(ui.layout_front);
      side_view = new qrviz(ui.layout_side);
      top_view = new qrviz(ui.layout_top);


		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "ros_qt_demo");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "ros_qt_demo");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
  /*if(roscore_cmd)
  {
        roscore_cmd->close();
        roscore_cmd->waitForFinished();
  }
  */
	WriteSettings();
	QMainWindow::closeEvent(event);

}

}  // namespace ros_qt_demo


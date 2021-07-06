#include "../include/ros_qt_demo/qrviz.hpp"
#include <QDebug>

qrviz::qrviz(QVBoxLayout* layout)
{
  // create rviz panel
  render_panel_ = new rviz::RenderPanel();
  layout->addWidget(render_panel_);

  manager_ = new rviz::VisualizationManager(render_panel_);
  //?
  ROS_ASSERT(manager_ != NULL);
  //init render panel, zoom in/out
  render_panel_->initialize(manager_->getSceneManager(), manager_);

  manager_->initialize();
  manager_->removeAllDisplays();
  manager_->startUpdate();

}

void qrviz::Set_FixedFrame(QString Frame_name) {
  manager_->setFixedFrame(Frame_name);
  qDebug() << manager_->getFixedFrame();
}

void qrviz::Display_Grid(int Cell_Count, QColor color, bool enable) {

  if(Grid_ != nullptr) {
    delete Grid_;
    Grid_ = nullptr;
  }

  Grid_ = manager_->createDisplay("rviz/Grid", "myGrid", enable);
  //set CellCount
  Grid_->subProp("Plane Cell Count")->setValue(Cell_Count);
  //set Color
  Grid_->subProp("Color")->setValue(color);
  ROS_ASSERT(Grid_ != nullptr);
}

void qrviz::Display_TF(bool enable) {
  if(TF_ != nullptr) {
    delete TF_;
    TF_ = nullptr;
  }

  TF_ = manager_->createDisplay("rviz/TF", "myTF", enable);
  //set MarkerScale
  TF_->subProp("Marker Scale")->setValue(0.4);

  ROS_ASSERT(TF_ != nullptr);
}

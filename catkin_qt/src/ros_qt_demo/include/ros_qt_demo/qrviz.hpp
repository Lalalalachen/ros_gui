#ifndef QRVIZ_HPP
#define QRVIZ_HPP

#include <QObject>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <QVBoxLayout>

class qrviz
{
public:
  qrviz(QVBoxLayout* layout);
  void Set_FixedFrame(QString Frame_name);
  void Display_Grid(int Cell_Count, QColor color, bool enable);
  void Display_TF(bool enable);
  void Display_RobotModel(bool enable);
private:
  rviz::RenderPanel* render_panel_;
  rviz::VisualizationManager* manager_;
  rviz::Display* Grid_ = nullptr;
  rviz::Display* TF_ = nullptr;
  rviz::Display* RM_ = nullptr;
};

#endif // QRVIZ_HPP

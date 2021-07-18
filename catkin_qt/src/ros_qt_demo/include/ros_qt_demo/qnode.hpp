/**
 * @file /include/ros_qt_demo/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ros_qt_demo_QNODE_HPP_
#define ros_qt_demo_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
// to subscribe image topic
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <QImage>
// to subscribe /tf
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_qt_demo {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
  // image Subscribe pushbutton interface function
  void sub_image(QString topic_name);
  // TF play bag file pushbutton interface function
  enum Status {
    Reach,
    Move,
    Wait
  };
  QStringListModel* statusModel() { return &status_model; }
  void status( const Status &status, const std::string &st);
  double tx = 0.5; // translation error
  double ty = 0.5; // translation error
  double tz = 0.5; // translation error
  double rx = 0.5; // rotation error
  double ry = 0.5; // rotation error
  double rz = 0.5; // rotation error
  double rw = 0.5; // rotation error
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void image_vel(QImage);
    void statusUpdated();


private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;
    QStringListModel status_model;
  ros::Subscriber chatter_subscriber;
  image_transport::Subscriber image_sub;
  tf2_ros::Buffer buffer;
  //tf2_ros::TransformListener listener(const tf2_ros::Buffer& buffer);


  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  QImage Mat2QImage(cv::Mat const& src);
  void chatter_callback(const std_msgs::String &msg);
};

}  // namespace ros_qt_demo

#endif /* ros_qt_demo_QNODE_HPP_ */

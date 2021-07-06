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

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;
  ros::Subscriber chatter_subscriber;
  image_transport::Subscriber image_sub;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  QImage Mat2QImage(cv::Mat const& src);
  void chatter_callback(const std_msgs::String &msg);
};

}  // namespace ros_qt_demo

#endif /* ros_qt_demo_QNODE_HPP_ */

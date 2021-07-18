/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ros_qt_demo/qnode.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_qt_demo {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

// initialize ros node by environment variables
bool QNode::init() {
	ros::init(init_argc,init_argv,"ros_qt_demo");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  chatter_subscriber = n.subscribe("chatter", 1000, &QNode::chatter_callback, this);
  // to subscribe TF msg
  tf2_ros::TransformListener listener(buffer);
  // call QNode::run() to start Qt thread
	start();
	return true;
}

// initiallize ros node by text line info
bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"ros_qt_demo");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  chatter_subscriber = n.subscribe("chatter", 1000, &QNode::chatter_callback, this);
  // to subscribe TF msg
  tf2_ros::TransformListener listener(buffer);
  // call QNode::run() to start Qt thread
	start();
	return true;
}



// define image Subscribe pushbutton
void QNode::sub_image(QString topic_name) {
  ros::NodeHandle n;
  image_transport::ImageTransport it_(n);
  image_sub = it_.subscribe(topic_name.toStdString(), 1000, &QNode::imageCallback, this);
}

// define callback function - subscribe image
void QNode::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  QImage im = Mat2QImage(cv_ptr->image);
  emit image_vel(im);
}

// Mat type to QImage type
QImage QNode::Mat2QImage(cv::Mat const& src) {
  QImage dest(src.cols, src.rows, QImage::Format_ARGB32);

  const float scale = 255.0;

  if (src.depth() == CV_8U) {
    if (src.channels() == 1) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          int level = src.at<quint8>(i, j);
          dest.setPixel(j, i, qRgb(level, level, level));
        }
      }
    } else if (src.channels() == 3) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
          dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
        }
      }
    }
  } else if (src.depth() == CV_32F) {
    if (src.channels() == 1) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          int level = scale * src.at<float>(i, j);
          dest.setPixel(j, i, qRgb(level, level, level));
        }
      }
    } else if (src.channels() == 3) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
          dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
        }
      }
    }
  }

  return dest;
}

// define callback function
void QNode::chatter_callback(const std_msgs::String &msg) {
  log(Info, "I receive " + msg.data);

}

// define a show status function


void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;

		ss << "hello world " << count;
		msg.data = ss.str();
    // publish msg
		chatter_publisher.publish(msg);
    // use log to show info in logging window
		log(Info,std::string("I sent: ")+msg.data);
    // loop waiting callback function

    // evaluate tf msg
    std::stringstream st;
    tf2_ros::TransformListener listener(buffer);
    try {
      geometry_msgs::TransformStamped tfs = buffer.lookupTransform("camera_target_position",
                                                                   "camera",
                                                                   ros::Time(0));

      //double t_error = 1.0; // translation error
      //double r_error = 1.0; // rotation error
      if(abs(tfs.transform.translation.x) < tx &&
         abs(tfs.transform.translation.y) < ty &&
         abs(tfs.transform.translation.z) < tz &&
         abs(tfs.transform.rotation.x) < rx &&
         abs(tfs.transform.rotation.y) < ry &&
         abs(tfs.transform.rotation.z) < rz &&
         abs(tfs.transform.rotation.w) < rw ) {
        st << "Reach the target!" << count;
        status(Reach, st.str());
      }
      else {
        st << "Keep move" << count;
        status(Move, st.str());
      }

    } catch (const std::exception& e) {
      ROS_INFO("Wrong info:%s",e.what());
      st << "Wait for change" << count;
      status(Wait, st.str());
    }


		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

//status
void QNode::status(const Status &status, const std::string &st) {
  status_model.insertRows(status_model.rowCount(),1);
  std::stringstream status_model_st;
  switch (status) {
  case(Reach) : {
    status_model_st << st;
    break;
  }
  case(Move) : {
    status_model_st << st;
    break;
  }
  case(Wait) : {
    status_model_st << st;
    break;
  }
  }
  QVariant new_row(QString(status_model_st.str().c_str()));
  status_model.setData(status_model.index(status_model.rowCount()-1),new_row);
  Q_EMIT statusUpdated(); // used to readjust the scrollbar
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace ros_qt_demo

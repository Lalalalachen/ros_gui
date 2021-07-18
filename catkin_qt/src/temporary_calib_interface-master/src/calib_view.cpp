#include <string>

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <calibration_msgs/observation.h>
#include <calibration_msgs/image_point.h>

class calibration_view
{
    ros::NodeHandle nh;
    image_transport::ImageTransport transport;
    std::string m_camera_name_left, m_camera_name_right;
    image_transport::Publisher pub_left, pub_right, pub_detected_obs;
    ros::Subscriber sub_used_obs, sub_detected_obs ;

public:
    calibration_view(std::string camera_name_left,
                     std::string camera_name_right) :
        transport(nh),
        m_camera_name_left(camera_name_left),
        m_camera_name_right(camera_name_right){
        sub_used_obs = nh.subscribe("uesd_observations", 10, &calibration_view::callback_used, this);
        sub_detected_obs = nh.subscribe("detected_observations", 10, &calibration_view::callback_detected, this);
        pub_detected_obs = transport.advertise("vis_detected_obs", 10);
        if (m_camera_name_left != "")
            pub_left = transport.advertise("vis_" + m_camera_name_left + "_obs", 10);

        if (m_camera_name_right != "")
            pub_right = transport.advertise("vis_" + m_camera_name_right + "_obs", 10);
    }

    void callback_detected(const calibration_msgs::observation &obs)
    {
        // Draw markers to debugging image
        cv::Mat debugging_image;
        try {
            if (obs.image.encoding == "rgb8"){
                debugging_image = cv_bridge::toCvCopy(obs.image)->image;
            } else if (obs.image.encoding == "mono8") {
                cv::cvtColor(cv_bridge::toCvCopy(obs.image)->image,debugging_image,cv::COLOR_GRAY2RGB);
            } else {
                ROS_ERROR("Unknown image encoding: %s", obs.image.encoding.c_str());
            }
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        std::string label;
        for (auto const &p: obs.observed_points){
            cv::drawMarker(debugging_image,cv::Point2d(p.x,p.y),cv::Scalar(0,255,0),cv::MARKER_CROSS,60,8);
            label = std::to_string(p.id);
            cv::putText(debugging_image, label, cv::Point(p.x, p.y), cv::FONT_HERSHEY_PLAIN, 4, cv::Scalar(0,0,255), 3);
        }
        label = obs.camera_name + " " + std::to_string(obs.position_index);
        cv::putText(debugging_image, label, cv::Point(10, 80), cv::FONT_HERSHEY_PLAIN, 6, cv::Scalar(0,0,255), 4);

        // Publish image with additional information
        pub_detected_obs.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", debugging_image).toImageMsg());
    }

    void callback_used(const calibration_msgs::observation &obs)
    {
        // Draw markers to debugging image
        cv::Mat debugging_image;
        try {
            if (obs.image.encoding == "rgb8"){
                debugging_image = cv_bridge::toCvCopy(obs.image)->image;
            } else if (obs.image.encoding == "mono8") {
                cv::cvtColor(cv_bridge::toCvCopy(obs.image)->image,debugging_image,cv::COLOR_GRAY2RGB);
            } else {
                ROS_ERROR("Unknown image encoding: %s", obs.image.encoding.c_str());
            }
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        std::string label;
        for (auto const &p: obs.observed_points){
            cv::drawMarker(debugging_image,cv::Point2d(p.x,p.y),cv::Scalar(0,255,0),cv::MARKER_CROSS,60,8);
            label = std::to_string(p.id);
            cv::putText(debugging_image, label, cv::Point(p.x, p.y), cv::FONT_HERSHEY_PLAIN, 4, cv::Scalar(0,0,255), 3);
        }
        label = obs.camera_name + " " + std::to_string(obs.position_index);
        cv::putText(debugging_image, label, cv::Point(10, 80), cv::FONT_HERSHEY_PLAIN, 6, cv::Scalar(0,0,255), 4);

        // Publish image with additional information
        if (obs.camera_name == m_camera_name_left)
            pub_left.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", debugging_image).toImageMsg());

        if (obs.camera_name == m_camera_name_right)
            pub_right.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", debugging_image).toImageMsg());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calib_view");
    std::string l_camera_name, r_camera_name;

    if (argc == 2){
       l_camera_name = argv[1];
       r_camera_name = "";
    } else if (argc == 3) {
        l_camera_name = argv[1];
        r_camera_name = argv[2];
    } else {
        ROS_ERROR("Use: calib_view <camera_name_left> <camera_name_right>");
        return -1;
    }
    calibration_view view(l_camera_name, r_camera_name);
    ros::spin();
    return 0;
}

#include "../lib/visual_values.hpp"
#include "visual_functionalities/lib_recording.hpp"
#include "visual_functionalities/GetRealObjectState.h"

#include <boost/bind.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

// The visual_values structure is used by all call backs, main and service
Visual_values visual_values;

//get the pic variable filled, and show a circle and a rectangle around the marker
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    config_pic(msg, visual_values);
    config_and_detect_markers(visual_values);
    //show_marker(visual_values);
}

//get camera info to make the tracking accurate
void infoimageCb(const sensor_msgs::CameraInfoConstPtr msg){
    visual_values.set_info_msg(msg);
}

//get object position (x, y and z)
void depthimageCb(const sensor_msgs::ImageConstPtr& depth_msg){
    locate_object(depth_msg, visual_values);
}

bool get_real_object_state_service_callback(visual_functionalities::GetRealObjectState::Request &req,
                                            visual_functionalities::GetRealObjectState::Response &res,
                                            ros::NodeHandle& nh,
                                            image_transport::ImageTransport& it_){

    visual_values.get_camera_char().readFromXMLFile("../../misc/camera_param_baxter.xml");

    //set params in launch file to construct the transformation matrix
    nh.getParam("camera_pose", visual_values.get_camera_frame_pose());
    nh.getParam("camera_frame_choice", visual_values.get_camera_frame_choice());

    //subscribers
    image_transport::Subscriber in_image = it_.subscribe("/camera/rgb/image_rect_color", 1, imageCb);
    ros::Subscriber in_info_image = nh.subscribe<sensor_msgs::CameraInfoConstPtr>("/camera/rgb/camera_info", 1, infoimageCb);
    image_transport::Subscriber in_depth_image = it_.subscribe("/camera/depth_registered/sw_registered/image_rect", 1, depthimageCb);
    ros::Publisher image_publisher = nh.advertise<sensor_msgs::Image>("/robot/xdisplay", 1);

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();
    usleep(1e6);

    ros::Rate rate(50.0);
    rate.sleep();

    usleep(4e6);

    Eigen::Vector3d obj_pos = visual_values.get_object_position(req.object_index);
//    ROS_ERROR_STREAM("previous conversion obj_pos" << obj_pos);
    Eigen::Vector4d extended_vector;
    extended_vector << obj_pos[0],
                       obj_pos[1],
                       obj_pos[2],
                       1;
    std::vector<Eigen::Vector4d> vector;
    vector.push_back(extended_vector);
    std::vector<std::vector<double>> output_of_conversion;
    convert_whole_object_positions_vector(visual_values, vector, output_of_conversion);
    std::vector<double> obj_pos_converted = output_of_conversion[0];

//    ROS_ERROR_STREAM("Converted obj_pos" << obj_pos_converted);

    res.model_state = {obj_pos_converted[0],
                       obj_pos_converted[1],
                       obj_pos_converted[2],
                       0, 0, 0};

    ROS_INFO("Done.");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_real_object_state_service");
  ros::NodeHandle n;

  image_transport::ImageTransport it_(n);
  ros::ServiceServer service = n.advertiseService<
        visual_functionalities::GetRealObjectState::Request,
        visual_functionalities::GetRealObjectState::Response>("visual_functionalities/get_real_model_state",
                                                              boost::bind(get_real_object_state_service_callback, 
                                                              _1, _2, n, it_));
  ROS_INFO("Ready to get real object state.");
  ros::spin();

  return 0;
}

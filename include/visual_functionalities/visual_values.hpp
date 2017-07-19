#ifndef __PARAMETERS_HPP__
#define __PARAMETERS_HPP__
#include <ros/ros.h>
#include <iostream>
#include <cmath>        // std::abs

#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <image_processing/DescriptorExtraction.h>

#include <aruco/aruco.h>
#include <aruco/dictionary.h>
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>

#include <sensor_msgs/image_encodings.h>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/DigitalIOState.h>
#include <crustcrawler_core_msgs/EndpointState.h>

#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <Eigen/Core>
#include <moveit/move_group/move_group_context.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <yaml-cpp/yaml.h>

#include <visual_functionalities/object_detection_by_qr_code.h>
#include <visual_functionalities/object_qr_position.h>

struct Camera_values {
    XmlRpc::XmlRpcValue parameters;
    size_t number_of_markers = 2;
    std::map<int, geometry_msgs::PointStamped> objects_positions;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat raw_input_picture;
    sensor_msgs::ImageConstPtr rgb_msg, depth_msg;
    sensor_msgs::CameraInfoConstPtr camera_info_msg;
    std::vector<Eigen::Vector2i> marker_center;
    aruco::MarkerDetector marker_detector;
    aruco::CameraParameters camera_character;
    std::vector<aruco::Marker> markers;
    float marker_size = 0.04;

    std::shared_ptr<rgbd_utils::RGBD_to_Pointcloud> converter;
    sensor_msgs::PointCloud2 ptcl_msg;
    std::string child_frame, parent_frame;

    bool camera_topics_good = false;
};
class Visual_values{
public:
    Camera_values params;
    Visual_values(){
        //params.point_in_camera_frame.resize(params.number_of_points, 4);
        //params.point_in_robot_frame.resize(params.number_of_points, 4);
    }


    ///getters
    int get_number_of_markers(){
        return params.number_of_markers;
    }

    XmlRpc::XmlRpcValue& get_parameters(){
        return params.parameters;
    }

    std::map<int, geometry_msgs::PointStamped>& get_objects_positions_map(){
        return params.objects_positions;
    }

    geometry_msgs::PointStamped& get_object_position(int id){
        return params.objects_positions.find(id)->second;
    }

    cv_bridge::CvImagePtr& get_cvt(){
        return params.cv_ptr;
    }

    cv::Mat& get_raw_original_picture(){
        return params.raw_input_picture;
    }

    sensor_msgs::ImageConstPtr& get_rgb_msg(){
        return params.rgb_msg;
    }

    sensor_msgs::ImageConstPtr& get_depth_msg(){
        return params.depth_msg;
    }

    sensor_msgs::CameraInfoConstPtr& get_camera_info_msg(){
        return params.camera_info_msg;
    }

    std::vector<Eigen::Vector2i>& get_marker_centers(){
        return params.marker_center;
    }

    aruco::CameraParameters& get_camera_character(){
        return params.camera_character;
    }

    aruco::MarkerDetector& get_aruco_marker_detector(){
        return params.marker_detector;
    }

    std::vector<aruco::Marker>& get_markers(){
        return params.markers;
    }

    float& get_marker_size(){
        return params.marker_size;
    }

    rgbd_utils::RGBD_to_Pointcloud& get_rgb_cloud_converter(){
        return *params.converter;
    }

    sensor_msgs::PointCloud2& get_pointcloud_msg(){
        return params.ptcl_msg;
    }

    bool get_camera_topics_status(){
        return params.camera_topics_good;
    }

    std::string& get_child_frame(){
        return params.child_frame;
    }

    std::string& get_parent_frame(){
        return params.parent_frame;
    }

    ///setters
    ///
    void set_parameters(XmlRpc::XmlRpcValue parameters){
        params.parameters = parameters;
    }

    void set_cvt(cv_bridge::CvImagePtr& cvimage){
        params.cv_ptr = cvimage;
    }

    void set_object_position(int id, geometry_msgs::PointStamped& object_position){
            params.objects_positions.insert ( std::pair<int, geometry_msgs::PointStamped>(id, object_position) );
    }

    void set_raw_original_picture(cv::Mat& picture){
        params.raw_input_picture = picture;
    }

    void set_rgb_msg(const sensor_msgs::ImageConstPtr& input_rgb_msg){
        params.rgb_msg = input_rgb_msg;
    }

    void set_depth_msg(sensor_msgs::ImageConstPtr& input_depth_msg){
        params.depth_msg = input_depth_msg;
    }

    void set_camera_info_msg(aruco::CameraParameters& camera_character){
        params.camera_character = camera_character;
    }

    void set_marker_center(std::vector<Eigen::Vector2i>& vector_marker_center){
        params.marker_center = vector_marker_center;
    }

    void set_camera_character(aruco::CameraParameters& camera_character){
        params.camera_character = camera_character;
    }

    void set_markers(std::vector<aruco::Marker>& markers){
        params.markers = markers;
    }

    void set_marker_size(float marker_size){
        params.marker_size = marker_size;
    }

    void set_rgb_cloud_converter(sensor_msgs::ImageConstPtr depth_msg,
                                 sensor_msgs::ImageConstPtr rgb_msg,
                                 sensor_msgs::CameraInfoConstPtr camera_info){
        params.converter.reset(new rgbd_utils::RGBD_to_Pointcloud(depth_msg, rgb_msg, camera_info));
    }

    void set_pointcloud_msg(sensor_msgs::PointCloud2 ptcl_msg){
        params.ptcl_msg = ptcl_msg;
    }

    void set_camera_topics_status(bool status){
        params.camera_topics_good = status;
    }

    void set_child_frame(std::string frame){
        params.child_frame = frame;
    }

    void set_parent_frame(std::string frame){
        params.parent_frame = frame;
    }
};

#endif

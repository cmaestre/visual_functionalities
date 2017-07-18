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

struct Camera_values {
    XmlRpc::XmlRpcValue parameters;
    size_t number_of_markers = 2;
    std::map<int, geometry_msgs::PointStamped> objects_positions;
    int number_of_points = 10, number_of_valid_point = 0;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat raw_input_picture;
    sensor_msgs::ImageConstPtr rgb_msg, depth_msg;
    sensor_msgs::CameraInfoConstPtr camera_info_msg;
    std::vector<Eigen::Vector2i> marker_center;
    double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0, epsilon = 0.01, the_rate = 50.0;
    Eigen::Matrix4d point_in_robot_frame, point_in_camera_frame;
    aruco::MarkerDetector marker_detector;
    aruco::CameraParameters camera_character;
    std::vector<aruco::Marker> markers;
    float marker_size = 0.04;

    std::shared_ptr<rgbd_utils::RGBD_to_Pointcloud> converter;
    sensor_msgs::PointCloud2 ptcl_msg;
    Eigen::MatrixX4d markers_positions_camera_frame, markers_positions_robot_frame;
    Eigen::Matrix4d transformation_matrix;
    Eigen::Vector3d transformation_RPY;
    tf::Quaternion transformation_quaternion;
    std::string camera_fram_pose;
    std::string camera_frame_choice, child_frame, parent_frame;

    std::vector<double> camera_rgb_optical_pose, camera_depth_optical_pose, camera_link_pose;
    Eigen::Matrix4d Trans_M;

    int8_t pressed;
    bool release = true, toggle = false, l_lower_button_pressed = false, camera_topics_good = false;

    std::string camera_param_path;
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

    double get_x_coordinate(){
        return params.x_coord;
    }

    double get_y_coordinate(){
        return params.y_coord;
    }

    double get_z_coordinate(){
        return params.z_coord;
    }

    Eigen::Vector3d get_marker_position(){
        Eigen::Vector3d marker_position;
        marker_position << params.x_coord, params.y_coord, params.z_coord;
        return marker_position;
    }

    Eigen::Matrix4d& get_point_in_robot_frame(){
        return params.point_in_robot_frame;
    }

    Eigen::Matrix4d& get_point_in_camera_frame(){
        return params.point_in_camera_frame;
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

    std::vector<double>& get_camera_depth_optical_pose(){
        return params.camera_depth_optical_pose;
    }

    rgbd_utils::RGBD_to_Pointcloud& get_rgb_cloud_converter(){
        return *params.converter;
    }

    sensor_msgs::PointCloud2& get_pointcloud_msg(){
        return params.ptcl_msg;
    }

    Eigen::MatrixX4d& get_markers_positions_camera_frame(){
        return params.markers_positions_camera_frame;
    }

    Eigen::MatrixX4d& get_markers_positions_robot_frame(){
        return params.markers_positions_robot_frame;
    }

    Eigen::Matrix4d& get_transformation_matrix(){
        return params.Trans_M ;
    }

    bool get_camera_topics_status(){
        return params.camera_topics_good;
    }

    int8_t& get_pressed(){
        return params.pressed;
    }

    bool& get_release(){
        return params.release;
    }

    bool& get_toggle(){
        return params.toggle;
    }

    bool& get_lower_botton_pressed(){
        return params.l_lower_button_pressed;
    }

    double& get_epsilon(){
        return params.epsilon;
    }

    double& get_the_rate(){
        return params.the_rate;
    }

    int& get_number_of_validated_points(){
        return params.number_of_valid_point;
    }

    std::string& get_camera_frame_pose(){
        return params.camera_fram_pose;
    }

    std::string& get_camera_frame_choice(){
        return params.camera_frame_choice;
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

    void set_x_coordinate(double x){
        params.x_coord = x;
    }

    void set_pressed(int8_t pressed){
        params.pressed = pressed;
    }

    void set_release(bool release){
        params.release = release;
    }

    void set_lower_button_pressed(bool l_lower_button_pressed){
        params.l_lower_button_pressed = l_lower_button_pressed;
    }

    void set_toggle(bool toggle){
        params.toggle = toggle;
    }

    void set_epsilon(double& epsilon){
        params.epsilon = epsilon;
    }

    void set_the_rate(double& the_rate){
        params.the_rate = the_rate;
    }

    void set_transformation_matrix(Eigen::Matrix4d& Trans_M){
        params.Trans_M = Trans_M;
    }

    void set_camera_rgb_optical_pose(std::vector<double>& pose){
        params.camera_rgb_optical_pose = pose;
    }

    void set_y_coordinate(double y){
        params.y_coord = y;
    }

    void set_z_coordinate(double z){
        params.z_coord = z;
    }

    void set_point_in_robot_frame(Eigen::Matrix4d& point){
        params.point_in_robot_frame = point;
    }

    void set_point_in_camera_frame(Eigen::Matrix4d& point){
        params.point_in_camera_frame = point;
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

    void set_marker_position_camera_frame(Eigen::MatrixX4d markers_positions){
        params.markers_positions_camera_frame = markers_positions;
    }

    void set_marker_position_robot_frame(Eigen::MatrixX4d markers_positions){
        params.markers_positions_robot_frame = markers_positions;
    }

    void set_camera_depth_optical_pose(std::vector<double>& pose){
        params.camera_depth_optical_pose = pose;
    }

    void set_camera_topics_status(bool status){
        params.camera_topics_good = status;
    }

    void set_number_of_validated_point(int i){
        params.number_of_valid_point = i;
    }

    void set_child_frame(std::string frame){
        params.child_frame = frame;
    }

    void set_parent_frame(std::string frame){
        params.parent_frame = frame;
    }
};

#endif

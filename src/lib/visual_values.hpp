#ifndef __SIM_PARAM_HPP__
#define __SIM_PARAM_HPP__
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <Eigen/Core>

#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>

#include <aruco/aruco.h>

#include <tf/tf.h>

struct Camera_values {
    ///////// OBJECT tracking variables
    // object position (x, y, z) in real world in robot base frame
    size_t number_of_markers = 2;
    cv_bridge::CvImagePtr cv_ptr;
    std::vector<Eigen::Vector3d> object_position_vector;


    cv::Mat pic;
    sensor_msgs::ImageConstPtr rgb_msg, depth_msg;
    sensor_msgs::CameraInfoConstPtr info_msg;
    std::vector<Eigen::Vector2i> marker_center_vector;


    aruco::MarkerDetector aruco_detector;
    aruco::CameraParameters camera_char;
    std::vector<aruco::Marker> markers_vector;
    std::vector<int> markers_id_vector;

    float marker_size = 0.05;

    std::string object_file_name = "";

    ////// ROBOT arm vairables
    // left end effector pose
    Eigen::VectorXd left_eef_pose_rpy;
    geometry_msgs::Pose left_eef_pose_quat;
    tf::Quaternion left_eef_rpy_orientation;

    std::string left_eef_trajectory_file_name = "";
    /////// SHARED variables
    int8_t pressed;
    bool release = true, toggle = false, l_lower_button_pressed = false;
    double epsilon = 0.01;
    std::string camera_param_path;
    double the_rate = 50.0;
    std::vector<double> camera_rgb_optical_pose, camera_depth_optical_pose, camera_link_pose;
    Eigen::Matrix4d Trans_M;
    std::string camera_fram_pose;
    std::string camera_frame_choice;
};

class Visual_values{

public:
    Camera_values camera_values;

    Visual_values(){
        //size setters
        camera_values.object_position_vector.resize(camera_values.number_of_markers);
        camera_values.marker_center_vector.resize(camera_values.number_of_markers);
        camera_values.markers_vector.resize(camera_values.number_of_markers);
        camera_values.markers_id_vector.resize(camera_values.number_of_markers);
    }

    void resize_vectors(int new_size){
        camera_values.markers_id_vector.resize(new_size);
    }

    //// Getters
    int get_number_of_markers(){
        return camera_values.number_of_markers;
    }

    cv_bridge::CvImagePtr& get_cv_pridge(){
        return camera_values.cv_ptr;
    }

    Eigen::Vector3d& get_object_position(int index){
        return camera_values.object_position_vector[index];
    }

    std::vector<Eigen::Vector3d>& get_object_position_vector(){
        return camera_values.object_position_vector;
    }

    cv::Mat& get_cv_image(){
        return camera_values.pic;
    }

    const sensor_msgs::ImageConstPtr& get_rgb_msg(){
        return camera_values.rgb_msg;
    }

    const sensor_msgs::ImageConstPtr& get_depth_msg(){
        return camera_values.depth_msg;
    }

    sensor_msgs::CameraInfoConstPtr& get_info_msg(){
        return camera_values.info_msg;
    }

    Eigen::Vector2i& get_marker_center(int index){
        return camera_values.marker_center_vector[index];
    }

    std::vector<Eigen::Vector2i>& get_marker_center_vector(){
        return camera_values.marker_center_vector;
    }

    aruco::MarkerDetector& get_aruco_detector(){
        return camera_values.aruco_detector;
    }

    aruco::CameraParameters& get_camera_char(){
        return camera_values.camera_char;
    }

    std::vector<aruco::Marker>& get_markers(){
        return camera_values.markers_vector;
    }

    std::vector<int>& get_markers_id_vector(){
        return camera_values.markers_id_vector;
    }

    float get_marker_size(){
        return camera_values.marker_size;
    }

    std::string& get_object_file_name(){
        return camera_values.object_file_name;
    }

    Eigen::VectorXd get_left_eef_pose_rpy(){
        return camera_values.left_eef_pose_rpy;
    }

    geometry_msgs::Pose& get_left_eef_pose_quat(){
        return camera_values.left_eef_pose_quat;
    }

    tf::Quaternion& get_left_eef_rpy_orientation(){
        return camera_values.left_eef_rpy_orientation;
    }

    std::string& get_left_eef_trajectory_file_name(){
        return camera_values.left_eef_trajectory_file_name;
    }

    int8_t& get_pressed(){
        return camera_values.pressed;
    }

    bool& get_release(){
        return camera_values.release;
    }

    bool& get_toggle(){
        return camera_values.toggle;
    }

    bool& get_lower_botton_pressed(){
        return camera_values.l_lower_button_pressed;
    }

    double& get_epsilon(){
        return camera_values.epsilon;
    }

    double& get_the_rate(){
        return camera_values.the_rate;
    }

    std::string& get_camera_param_path(){
        return camera_values.camera_param_path;
    }

    std::vector<double>& get_camera_depth_optical_pose(){
        return camera_values.camera_depth_optical_pose;
    }

    std::vector<double>& get_camera_rgb_optical_pose(){
        return camera_values.camera_rgb_optical_pose;
    }

    std::vector<double>& get_camera_link_pose(){
        return camera_values.camera_link_pose;
    }

    Eigen::Matrix4d& get_transformation_matrix(){
        return camera_values.Trans_M ;
    }

    std::string& get_camera_frame_pose(){
        return camera_values.camera_fram_pose;
    }

    std::string& get_camera_frame_choice(){
        return camera_values.camera_frame_choice;
    }

    //// Setters
    void set_number_of_marker(int number){
        camera_values.number_of_markers = number;
    }

    void set_cv_pridged(cv_bridge::CvImagePtr& cv_ptr){
        camera_values.cv_ptr = cv_ptr;
    }

    void set_object_position(Eigen::Vector3d& object_position, int index){
        camera_values.object_position_vector[index] = object_position;
    }

    void set_cv_image(cv::Mat& cv_image){
        camera_values.pic = cv_image;
    }

    void set_rgb_msg(const sensor_msgs::ImageConstPtr& rgb_msg){
        camera_values.rgb_msg = rgb_msg;
        //camera_values.rgb_msg.reset(*rgb_msg);
    }

    void set_depth_msg(const sensor_msgs::ImageConstPtr& depth_msg){
        camera_values.depth_msg = depth_msg;
        //camera_values.depth_msg.reset(*depth_msg);
    }

    void set_info_msg(const sensor_msgs::CameraInfoConstPtr info_msg){
        camera_values.info_msg = info_msg;
    }

    void set_marker_size(float& marker_size){
        camera_values.marker_size = marker_size;
    }

    void set_marker_center(Eigen::Vector2i& marker_center, int index){
        camera_values.marker_center_vector[index] = marker_center;
    }

    void set_camera_char(aruco::CameraParameters& camera_char){
        camera_values.camera_char = camera_char;
    }

    void set_markers_id_vector(int marker_id, int marker_index){
        camera_values.markers_id_vector[marker_index] = marker_id;
    }

    void set_markers(std::vector<aruco::Marker>& markers){
        camera_values.markers_vector = markers;
    }

    void set_object_file_name(std::string file_name){
        camera_values.object_file_name = file_name;
    }

    void set_left_eef_pose_rpy(Eigen::VectorXd& left_eef_pose_rpy){
        camera_values.left_eef_pose_rpy = left_eef_pose_rpy;
    }

    void set_left_eef_pose_quat(geometry_msgs::Pose& left_eef_pose_quat){
        camera_values.left_eef_pose_quat = left_eef_pose_quat;
    }

    void set_left_eeft_rpy_orientation(tf::Quaternion& left_eef_rpy_orientation){
        camera_values.left_eef_rpy_orientation = left_eef_rpy_orientation;
    }


    void set_left_eef_trajectory_file_name(std::string file_name){
        camera_values.left_eef_trajectory_file_name = file_name;
    }

    void set_pressed(int8_t pressed){
        camera_values.pressed = pressed;
    }

    void set_camera_param_path(std::string& camera_param_path){
        camera_values.camera_param_path = camera_param_path;
    }

    void set_release(bool release){
        camera_values.release = release;
    }

    void set_toggle(bool toggle){
        camera_values.toggle = toggle;
    }

    void set_lower_button_pressed(bool l_lower_button_pressed){
        camera_values.l_lower_button_pressed = l_lower_button_pressed;
    }

    void set_epsilon(double& epsilon){
        camera_values.epsilon = epsilon;
    }

    void set_the_rate(double& the_rate){
        camera_values.the_rate = the_rate;
    }

    void set_camera_depth_optical_pose(std::vector<double>& pose){
        camera_values.camera_depth_optical_pose = pose;
    }

    void set_camera_rgb_optical_pose(std::vector<double>& pose){
        camera_values.camera_rgb_optical_pose = pose;
    }

    void set_camera_link_pose(std::vector<double>& pose){
        camera_values.camera_link_pose = pose;
    }

    void set_transformation_matrix(Eigen::Matrix4d& Trans_M){
        camera_values.Trans_M = Trans_M;
    }

    void set_camera_frame_pose(std::string pose){
        camera_values.camera_fram_pose = pose;
    }

    void set_camera_frame_choice(std::string frame_choice){
        camera_values.camera_frame_choice = frame_choice;
    }
};

#endif

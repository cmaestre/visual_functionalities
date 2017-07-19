#include <visual_functionalities/lib_recording.hpp>

//Convert object position from camera frame to robot frame
void lib_recording_functions::convert_object_position_to_robot_base(Visual_values& visual_values,
                                                                    Eigen::Vector3d& object_pose_in_camera_frame,
                                                                    Eigen::Vector3d& object_pose_in_robot_frame){
    tf::TransformListener listener;
    tf::StampedTransform stamped_transform;
    std::string child_frame = visual_values.get_child_frame();
    std::string parent_frame = visual_values.get_parent_frame();
    try{
        listener.lookupTransform(child_frame, parent_frame,
                                 ros::Time::now(), stamped_transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    geometry_msgs::PointStamped camera_point;
    geometry_msgs::PointStamped base_point;
    camera_point.header.frame_id = child_frame;

    //we'll just use the most recent transform available for our simple example
    camera_point.header.stamp = ros::Time();

    //just an arbitrary point in space
    camera_point.point.x = object_pose_in_camera_frame(0);
    camera_point.point.y = object_pose_in_camera_frame(1);
    camera_point.point.z = object_pose_in_camera_frame(2);

    try{
        listener.transformPoint(parent_frame, camera_point, base_point);
        ROS_INFO("%s: (%.2f, %.2f. %.2f) -----> %s: (%.2f, %.2f, %.2f) at time %.2f",
                 child_frame.c_str(),
                 camera_point.point.x, camera_point.point.y, camera_point.point.z,
                 parent_frame.c_str(),
                 base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", child_frame.c_str(), parent_frame.c_str(), ex.what());
    }
    object_pose_in_robot_frame << base_point.point.x,
            base_point.point.y,
            base_point.point.z;
}

void lib_recording_functions::convert_whole_object_positions_vector(Visual_values& visual_values,
                                                                    std::vector<std::vector<double>>& object_positions_vector,
                                                                    std::vector<std::vector<double>>& output_of_conversion){
    if(object_positions_vector.empty()){
        ROS_ERROR("THE TRANSFORMATION IS IMPOSSIBLE, EMPTY VECTOR");
        return;
    }
    tf::TransformListener listener;
    tf::StampedTransform stamped_transform;
    std::string child_frame = visual_values.get_child_frame();
    std::string parent_frame = visual_values.get_parent_frame();
    try{
        listener.lookupTransform(child_frame, parent_frame,
                                 ros::Time::now(), stamped_transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    std::vector<geometry_msgs::PointStamped> camera_point;
    std::vector<geometry_msgs::PointStamped> base_point;
    camera_point.resize(object_positions_vector.size());
    base_point.resize(object_positions_vector.size());

    for(size_t i = 0; i < camera_point.size(); i++){
        camera_point[i].header.frame_id = child_frame;

        //we'll just use the most recent transform available for our simple example
        camera_point[i].header.stamp = ros::Time();

        camera_point[i].point.x = object_positions_vector[i][0];
        camera_point[i].point.y = object_positions_vector[i][1];
        camera_point[i].point.z = object_positions_vector[i][2];

        try{
            listener.transformPoint(parent_frame, camera_point[i], base_point[i]);
            ROS_INFO("kinect2_rgb_optical_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                     camera_point[i].point.x, camera_point[i].point.y, camera_point[i].point.z,
                     base_point[i].point.x, base_point[i].point.y, base_point[i].point.z, base_point[i].header.stamp.toSec());
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_optical_frame\" to \"world\": %s", ex.what());
        }
        output_of_conversion[i] = {base_point[i].point.x, base_point[i].point.y, base_point[i].point.z};
    }
}

void lib_recording_functions::write_data(Visual_values& visual_values,
                                         std::vector<std::vector<double>>& object_positions_vector,
                                         std::ofstream& the_file){

    std::vector<std::vector<double>>::iterator outter_itr_object = object_positions_vector.begin();
    std::vector<double>::iterator inner_itr;
    for(int i = 0; i < visual_values.get_number_of_markers(); i++){ // obj position per object
        for(inner_itr = (*outter_itr_object).begin(); inner_itr != (*outter_itr_object).end(); inner_itr++)
            the_file << (*inner_itr) << ",";
        the_file << 0 << "," << 0 << "," << 0 << ","; // obj rotation
        ++outter_itr_object;
    }


    the_file << "\n";
}

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
                                                                    std::vector<Eigen::Vector4d>& object_positions_vector,
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

        camera_point[i].point.x = object_positions_vector[i](0);
        camera_point[i].point.y = object_positions_vector[i](1);
        camera_point[i].point.z = object_positions_vector[i](2);

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
                                         std::vector<std::vector<double>>& left_eef_trajectory,
                                         std::vector<std::vector<double>>& object_positions_vector, std::ofstream& the_file){
    std::vector<std::vector<double>>::iterator outter_itr;
    std::vector<std::vector<double>>::iterator outter_itr_object = object_positions_vector.begin();
    for(outter_itr = left_eef_trajectory.begin();
        outter_itr != left_eef_trajectory.end();
        outter_itr++){
        std::vector<double>::iterator inner_itr;
        for(inner_itr = (*outter_itr).begin(); inner_itr != (*outter_itr).end(); inner_itr++) // eef pos and rotation
            the_file << (*inner_itr) << ",";

        for(int i = 0; i < visual_values.get_number_of_markers(); i++){ // obj position per object
            for(inner_itr = (*outter_itr_object).begin(); inner_itr != (*outter_itr_object).end(); inner_itr++)
                the_file << (*inner_itr) << ",";
            the_file << 0 << "," << 0 << "," << 0 << ","; // obj rotation
            ++outter_itr_object;
        }
    }

    the_file << "\n";
}

//record marker position (object position) if changed in the specified file
void lib_recording_functions::record_traj_and_object_position(Visual_values& visual_values,
                                                              std::vector<std::vector<double>>& left_eef_trajectory,
                                                              ros::Publisher& image_pub,
                                                              std::ofstream& the_file){

    Eigen::VectorXd old_values(6);
    //old_values = visual_values.get_left_eef_pose_rpy();
    //ROS_ERROR_STREAM("here old values are: " << old_values);
    visual_values.set_pressed(false);
    visual_values.set_release(true);
    visual_values.set_toggle(false);
    std::string working_image_path = "/home/ghanim/git/catkin_ws/src/baxter_examples/share/images/working.jpg";
    std::string not_working_image_path = "/home/ghanim/git/catkin_ws/src/baxter_examples/share/images/finished.jpg";
    ros::Rate rate(visual_values.get_the_rate());
    rate.sleep();
    double time_now = 0.0;
    std::vector<Eigen::Vector4d> object_position_vector;
    std::vector<Eigen::Vector3d> prev_object_position_vector(visual_values.get_number_of_markers());
    int nb_iter = 0;
    while(ros::ok()){
        int followed_object_index = 0;
        //ROS_INFO("go go go");
        //dispaly_image(not_working_image_path, image_pub);
        if(visual_values.get_pressed() && visual_values.get_lower_botton_pressed()){
            if(visual_values.get_release()){
                visual_values.set_release(false);
                visual_values.set_toggle(true);
            }

            //wait till there is no (NaN values)
            //            if (visual_values.get_number_of_markers() == 1) {
            //            while(visual_values.get_object_position(followed_object_index)(0) != visual_values.get_object_position(followed_object_index)(0) ||
            //                  visual_values.get_object_position(followed_object_index)(1) != visual_values.get_object_position(followed_object_index)(1) ||
            //                  visual_values.get_object_position(followed_object_index)(2) != visual_values.get_object_position(followed_object_index)(2))
            //                ROS_ERROR("I am in waiting limbo for 1 object...........");
            //            }
            //            else if (visual_values.get_number_of_markers() == 2) {
            //                while(visual_values.get_object_position(0)(0) != visual_values.get_object_position(0)(0) ||
            //                      visual_values.get_object_position(0)(1) != visual_values.get_object_position(0)(1) ||
            //                      visual_values.get_object_position(0)(2) != visual_values.get_object_position(0)(2) ||
            //                      visual_values.get_object_position(1)(0) != visual_values.get_object_position(1)(0) ||
            //                      visual_values.get_object_position(1)(1) != visual_values.get_object_position(1)(1) ||
            //                      visual_values.get_object_position(1)(2) != visual_values.get_object_position(1)(2))
            //                    ROS_ERROR_STREAM("I am in waiting limbo for 2 objects...........");
            //            } else{
            //                ROS_ERROR("record_traj_and_object_position - wrong number of objects");
            //                exit(-1);
            //            }

            //get current eef pose
            Eigen::VectorXd current_values(6);
            //current_values = visual_values.get_left_eef_pose_rpy();
            if ((current_values - old_values).norm() > visual_values.get_epsilon()){
                time_now = ros::Time::now().toSec();

                std::vector<double> inner_left_traj;
                inner_left_traj.push_back(current_values(0));
                inner_left_traj.push_back(current_values(1));
                inner_left_traj.push_back(current_values(2));
                inner_left_traj.push_back(current_values(3));
                inner_left_traj.push_back(current_values(4));
                inner_left_traj.push_back(current_values(5));
                left_eef_trajectory.push_back(inner_left_traj);
                old_values = current_values;

                //while saving end effector changes register object position concurrently
                for(int i = 0; i < visual_values.get_number_of_markers(); i++){
                    Eigen::Vector3d current_obj_pos;
                    current_obj_pos << visual_values.get_object_position(0).point.x, visual_values.get_object_position(0).point.y, visual_values.get_object_position(0).point.z;
                    // If NaN get previous value
                    ROS_ERROR_STREAM("\n\n iteration" << nb_iter << " object ID " << i);
                    ROS_ERROR_STREAM(current_obj_pos[0] << " " << current_obj_pos[1] << " " << current_obj_pos[2]);
                    //                    ROS_ERROR_STREAM(prev_object_position_vector[i][0] << " " << prev_object_position_vector[i][1] << " " << prev_object_position_vector[i][2]);
                    //                    if (nb_iter > 0){
                    //                        for (int pos_coord = 0; pos_coord < 3; pos_coord++){
                    //                            ROS_ERROR_STREAM("resta " << std::abs(current_obj_pos[pos_coord] - prev_object_position_vector[i][pos_coord]));
                    //                            if (std::abs(current_obj_pos[pos_coord] - prev_object_position_vector[i][pos_coord]) > 0.01){
                    //                                current_obj_pos[pos_coord] = prev_object_position_vector[i][pos_coord];
                    //                                ROS_ERROR_STREAM("record_traj_and_object_position - obj pos coord " << i << " " << pos_coord << " problem");
                    //                            }
                    //                        }
                    //                    }

                    Eigen::Vector4d object_position_in_robot_frame;
                    object_position_in_robot_frame << current_obj_pos, 1;
                    object_position_vector.push_back(object_position_in_robot_frame);
                    prev_object_position_vector[i][0] = current_obj_pos[0];
                    prev_object_position_vector[i][1] = current_obj_pos[1];
                    prev_object_position_vector[i][2] = current_obj_pos[2];
                }
                ROS_ERROR_STREAM("this iteration duration is: " << ros::Time::now().toSec() - time_now);
            }
            nb_iter++;
        }
        else{
            if(visual_values.get_toggle()){
                ROS_ERROR("I need two buttons to be pressed simultaneously ...........");
                visual_values.set_release(true);
                visual_values.set_lower_button_pressed(false);

            }
        }
        if(visual_values.get_toggle() && !visual_values.get_lower_botton_pressed()){
            //left_eef_trajectory_file << "\n";
            //object_file << "\n";
            visual_values.set_toggle(false);
            std::vector<std::vector<double>> output_of_conversion;
            convert_whole_object_positions_vector(visual_values, object_position_vector, output_of_conversion);
            write_data(visual_values, left_eef_trajectory, output_of_conversion, the_file);
            left_eef_trajectory.clear();
            object_position_vector.clear();
        }
        rate.sleep();
    }
}

#include <visual_functionalities/lib_recording.hpp>

//Convert object position from camera frame to robot frame
void lib_recording_functions::convert_object_position_to_robot_base(Eigen::Vector3d& object_pose_in_camera_frame, Eigen::Vector3d& object_pose_in_robot_frame){
    tf::TransformListener listener;
    tf::StampedTransform stamped_transform;
    //std::string child_frame = "/camera_depth_optical_frame";
    std::string child_frame = "/camera_rgb_optical_frame";
    std::string parent_frame = "/world";
    /*try{
        listener.lookupTransform(child_frame, parent_frame,
                                 ros::Time::now(), stamped_transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }*/

    geometry_msgs::PointStamped camera_point;
    geometry_msgs::PointStamped base_point;
    camera_point.header.frame_id = child_frame;

    //we'll just use the most recent transform available for our simple example
    camera_point.header.stamp = ros::Time();

    //just an arbitrary point in space
    camera_point.point.x = object_pose_in_camera_frame(0);
    camera_point.point.y = object_pose_in_camera_frame(1);
    camera_point.point.z = object_pose_in_camera_frame(2);

    ros::Duration my_duration(0.03);
    try{

        //listener.waitForTransform(parent_frame, child_frame, ros::Time::now(), my_duration);
        listener.transformPoint(parent_frame, camera_point, base_point);

        ROS_INFO("camera_depth_optical_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                 camera_point.point.x, camera_point.point.y, camera_point.point.z,
                 base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_optical_frame\" to \"world\": %s", ex.what());
    }
    object_pose_in_robot_frame << base_point.point.x,
            base_point.point.y,
            base_point.point.z;
}

//Display some images on baxter screen to reflect what he is doing
void lib_recording_functions::dispaly_image(std::string path, ros::Publisher& image_pub){
    cv::Mat img = cv::imread(path);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    image_pub.publish(msg);
}

void lib_recording_functions::set_camera_poses_and_transformation(Visual_values& visual_values){
    tf::Quaternion my_angles;
    //from roslaunch file given pose construct a vector with x, y, z, roll, pitch and yaw
    std::vector<double> camera_pose;
    ROS_ERROR_STREAM("angles are: " << visual_values.get_camera_frame_pose() );
    std::stringstream ss (visual_values.get_camera_frame_pose());
    string field;
    while (getline( ss, field, ' ' ))
    {
        // for each field we wish to convert it to a double
        // (since we require that the CSV contains nothing but floating-point values)
        stringstream fs( field );
        double f = 0.0;  // (default value is 0.0)
        fs >> f;

        // add the newly-converted field to the end of the record
        camera_pose.push_back( f );
    }

    if(strcmp(visual_values.get_camera_frame_choice().c_str(), "rgb"))
        visual_values.set_camera_rgb_optical_pose(camera_pose);
    else if(strcmp(visual_values.get_camera_frame_choice().c_str(), "depth"))
            visual_values.set_camera_depth_optical_pose(camera_pose);
    else
        ROS_WARN("please specify in the your launch file a parameter with the name (camera_frame) with a value equal to: 'rgb' or 'depth'");

    my_angles.setRPY(camera_pose[3], camera_pose[4], camera_pose[5]);
    tf::Matrix3x3 rotation_matrix(my_angles);
    Eigen::Matrix4d trans_m;
    trans_m << rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], camera_pose[0],
            rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], camera_pose[1],
            rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], camera_pose[2],
                                0,                     0,                     0,              1;
    visual_values.set_transformation_matrix(trans_m);
    ROS_INFO_STREAM("and the transformation matrix from params is: \n" << visual_values.get_transformation_matrix());
}

void lib_recording_functions::convert_whole_object_positions_vector(Visual_values& visual_values,
                                           std::vector<Eigen::Vector4d>& object_positions_vector,
                                           std::vector<std::vector<double>>& output_of_conversion){
    set_camera_poses_and_transformation(visual_values);
    std::vector<Eigen::Vector4d>::iterator itr;
    for(itr = object_positions_vector.begin(); itr != object_positions_vector.end(); itr++){
        Eigen::Vector4d opv = visual_values.get_transformation_matrix() * (*itr);
        std::vector<double> inner_output;
        inner_output.push_back(opv(0));
        inner_output.push_back(opv(1));
        inner_output.push_back(opv(2));
        output_of_conversion.push_back(inner_output);
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
                    current_obj_pos << visual_values.get_object_position().point.x, visual_values.get_object_position().point.y, visual_values.get_object_position().point.z;
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

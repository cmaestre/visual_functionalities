#include <visual_functionalities/visual_values.hpp>


namespace lib_recording_functions {
/*Convert object position from camera frame to robot frame
 * input: object position in camera frame, and a vector to hold the transformation into robot frame
 * return: nothing but it record the trajectory and object position in relevant files
 * */
void convert_object_position_to_robot_base(Visual_values& visual_values,
                                           Eigen::Vector3d& object_pose_in_camera_frame,
                                           Eigen::Vector3d& object_pose_in_robot_frame);

/*Convert a vector of object positions from camera frame to robot frame
 * input: object position in camera frame, and a vector to hold the transformation into robot frame
 * return: nothing but it record the trajectory and object position in relevant files
 * */
void convert_whole_object_positions_vector(Visual_values &visual_values,
                                           std::vector<Eigen::Vector4d>& object_positions_vector,
                                           std::vector<std::vector<double> > &output_of_conversion);
void write_data(Visual_values& visual_values,
                std::vector<std::vector<double>>& left_eef_trajectory_and_object_vector,
                std::vector<std::vector<double>>& object_positions_vector, std::ofstream& the_file);

/*Recording method that uses all relevant visual_values to control starting/stopping of recording
 * input: the Visual_values class
 * return: nothing but it record the trajectory and object position in relevant files
 * */
void record_traj_and_object_position(Visual_values& visual_values,
                                     std::vector<std::vector<double> > &left_eef_trajectory,
                                     ros::Publisher &image_pub,
                                     ofstream &the_file);
}

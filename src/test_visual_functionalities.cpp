#include <visual_functionalities/visual_functionalities.hpp>


int main(int argc, char** argv){
    ros::init(argc, argv, "test_visual_functionalities");
    ros::NodeHandle n;

    visual_functionalities::VISUAL_FUNCTIONALITIES::Ptr visual_class;
    visual_class.reset(new visual_functionalities::VISUAL_FUNCTIONALITIES);

    ros::Rate rate(50);

    while(ros::ok()){
        visual_class->show_image();
//        ros::waitForShutdown();
//        ROS_INFO("TEST: Trying to locate object");
//        ros::spinOnce();
//        visual_class->get_object_position_cb();

//        if(!visual_class->get_global_parameters().get_marker_centers().empty()){
//            ROS_INFO_STREAM("TEST: Found an object and its position is: "
//                            << visual_class->get_global_parameters().get_object_position().point.x << ", "
//                            << visual_class->get_global_parameters().get_object_position().point.y << ", "
//                            << visual_class->get_global_parameters().get_object_position().point.z);

//            ROS_WARN("TEST: Press Enter to try again");
//            std::cin.ignore();
//        }
        rate.sleep();
    }
}

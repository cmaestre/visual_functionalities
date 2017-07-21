#include <visual_functionalities/visual_functionalities.hpp>


int main(int argc, char** argv){
    ros::init(argc, argv, "test_visual_functionalities");
    ros::NodeHandle n;

    XmlRpc::XmlRpcValue parameters;
    n.getParam("/visual_functionalities_parameters", parameters);
    std::string detection_method = std::string(parameters["detection_method"]);
    visual_functionalities::VISUAL_FUNCTIONALITIES::Ptr visual_class;
    visual_class.reset(new visual_functionalities::VISUAL_FUNCTIONALITIES);

    ros::Rate rate(50);

    while(ros::ok()){
        if(strcmp(detection_method.c_str(), "qr_code") == 0)
            visual_class->show_image_qr();
        if(strcmp(detection_method.c_str(), "blobs") == 0)
            visual_class->show_image_blob();
        rate.sleep();
    }
}

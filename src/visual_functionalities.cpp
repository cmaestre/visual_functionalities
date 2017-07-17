#include <visual_functionalities/visual_functionalities.hpp>

using namespace visual_functionalities;

void VISUAL_FUNCTIONALITIES::init(){
    ROS_INFO("VISUAL_FUNCTIONALITIES: initializing ......");
    _nh_visual_functionalities.getParam("/visual_functionalities_parameters", _global_parameters.get_parameters());
    _global_parameters.set_marker_size(std::stof(_global_parameters.get_parameters()["marker_size"]));

    //set the robot
    std::string robot_name = std::string(_global_parameters.get_parameters()["robot"]);
    std::string camera_name = std::string(_global_parameters.get_parameters()["camera"]);
    if(strcmp(robot_name.c_str(), "baxter") == 0){
        _robot.reset(new BAXTER);
        _robot->init();
    }
    else if(strcmp(robot_name.c_str(), "crustcrawler") == 0){
        _robot.reset(new CRUSTCRAWLER);
        _robot->init();
    }

    //set the camera
    if(strcmp(camera_name.c_str(), "kinect_v2") == 0){
        _camera.reset(new CAMERA_kinect_v2);
        _camera->init();
    }
    else if(strcmp(camera_name.c_str(), "kinect_freenect") == 0){
        _camera.reset(new CAMERA_kinect_freenect);
        _camera->init();
    }
    else if(strcmp(camera_name.c_str(), "kinect_openni") == 0){
        _camera.reset(new CAMERA_kinect_openni);
        _camera->init();
    }

    //set number of points to calibrate
    _visual_functionalities_spinner.reset(new ros::AsyncSpinner(1));
    _visual_functionalities_spinner->start();
    ROS_INFO("VISUAL_FUNCTIONALITIES: initialized");
}

void VISUAL_FUNCTIONALITIES::get_object_position(){
    ROS_INFO("VISUAL_FUNCTIONALITIES: at get object position");
    //while(!_global_parameters.get_camera_topics_status());
    _global_parameters.get_rgb_msg().reset(new sensor_msgs::Image(_camera->_syncronized_camera_sub->get_rgb()));
    _global_parameters.get_depth_msg().reset(new sensor_msgs::Image(_camera->_syncronized_camera_sub->get_depth()));
    _global_parameters.get_camera_info_msg().reset(new sensor_msgs::CameraInfo(_camera->_syncronized_camera_sub->get_rgb_info()));

    cv::namedWindow("ShowMarker",CV_WINDOW_AUTOSIZE);

    cv_bridge::CvImagePtr cv_ptr = _global_parameters.get_cvt();
    try
    {
        cv_ptr = cv_bridge::toCvCopy(_global_parameters.get_rgb_msg(), sensor_msgs::image_encodings::BGR8);
        _global_parameters.set_raw_original_picture(cv_ptr->image);
        _global_parameters.get_aruco_marker_detector().setDictionary("ARUCO");
        _global_parameters.get_aruco_marker_detector().detect(_global_parameters.get_raw_original_picture(),
                                                              _global_parameters.get_markers(),
                                                              _global_parameters.get_camera_character(),
                                                              0.1);
        _global_parameters.get_marker_centers().resize(_global_parameters.get_markers().size());

        ROS_WARN_STREAM("VISUAL_FUNCTIONALITIES: I found markers, number of them is: " << _global_parameters.get_markers().size());

        for(size_t i = 0; i < _global_parameters.get_markers().size(); i++){

            _global_parameters.get_markers()[i].draw(_global_parameters.get_raw_original_picture(), cv::Scalar(94.0, 206.0, 165.0, 0.0));

            _global_parameters.get_markers()[i].calculateExtrinsics(_global_parameters.get_marker_size(),
                                                                    _global_parameters.get_camera_character(),
                                                                    false);

            _global_parameters.get_marker_centers()[i] << (int) (_global_parameters.get_markers()[i][0].x + _global_parameters.get_markers()[i][2].x)/2,
                    (int) (_global_parameters.get_markers()[i][0].y + _global_parameters.get_markers()[i][2].y)/2;

            cv::circle(_global_parameters.get_raw_original_picture(), cv::Point(_global_parameters.get_marker_centers()[i](0),
                                                                                _global_parameters.get_marker_centers()[i](1)),
                                                                                10,
                                                                                CV_RGB(255, 0, 0));

                       _global_parameters.set_rgb_cloud_converter(_global_parameters.get_depth_msg(),
                                                                  _global_parameters.get_rgb_msg(),
                                                                  _global_parameters.get_camera_info_msg());
                    _global_parameters.set_pointcloud_msg(_global_parameters.get_rgb_cloud_converter().get_pointcloud());

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
                    input_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::fromROSMsg(_global_parameters.get_pointcloud_msg(), *input_cloud);
            for(size_t q = 0; q < _global_parameters.get_marker_centers().size(); q++){
                double x = _global_parameters.get_marker_centers()[q](0), y = _global_parameters.get_marker_centers()[q](1);
                pcl::PointXYZRGBA pt_marker = input_cloud->at(std::round(x) + std::round(y) * input_cloud->width);
                if(pt_marker.x == pt_marker.x && pt_marker.y == pt_marker.y && pt_marker.z == pt_marker.z){
                    ROS_WARN_STREAM("VISUAL_FUNCTIONALITIES: The marker 3D position is: " << pt_marker.x << ", " << pt_marker.y << ", " << pt_marker.z);
                    _global_parameters.get_markers_positions_camera_frame() << pt_marker.x, pt_marker.y, pt_marker.z , 1.0;

                }
            }
        }
//        ROS_WARN_STREAM("VISUAL_FUNCTIONALITIES: the supposed number is: " << _global_parameters.get_number_of_validated_points()++);
        cv::imshow("ShowMarker", _global_parameters.get_raw_original_picture());
        cv::waitKey(1);
    }
    catch(...)
    {
        ROS_ERROR("Something went wrong !!!");
        return;
    }
}


void CAMERA_kinect_v2::init(){
    ROS_INFO_STREAM("CAMERA_kinect_freenect: initializing, name space is: ");
    ROS_INFO_STREAM( _nh_camera.getNamespace());
    _syncronized_camera_sub.reset(new rgbd_utils::RGBD_Subscriber("/kinect2/qhd/camera_info",
                                                                  "/kinect2/qhd/image_color_rect",
                                                                  "/kinect2/qhd/camera_info",
                                                                  "/kinect2/qhd/image_depth_rect",
                                                                  _nh_camera));
    _camera_topics_sub = _nh_camera.subscribe<sensor_msgs::Image>("/kinect2/qhd/image_depth_rect", 1, &CAMERA_kinect_v2::camera_topics_start_publishing_cb, this);
    std::string camera_file_path;
    _nh_camera.getParam("/camera_file_path", camera_file_path);
    _global_parameters.get_camera_character().readFromXMLFile(camera_file_path);
    _camera_spinner.reset(new ros::AsyncSpinner(1));
    _camera_spinner->start();
}

void CAMERA_kinect_freenect::init(){
    _syncronized_camera_sub.reset(new rgbd_utils::RGBD_Subscriber("/camera/rgb/camera_info",
                                                                  "/camera/rgb/image_raw",
                                                                  "/camera/depth/camera_info",
                                                                  "/camera/depth/image_raw",
                                                                  _nh_camera));
    _camera_topics_sub = _nh_camera.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 1, &CAMERA_kinect_freenect::camera_topics_start_publishing_cb, this);
    std::string camera_file_path;
    _nh_camera.getParam("/camera_file_path", camera_file_path);
    _global_parameters.get_camera_character().readFromXMLFile(camera_file_path);
    _camera_spinner.reset(new ros::AsyncSpinner(1));
    _camera_spinner->start();
}

void CAMERA_kinect_openni::init(){
    ROS_INFO_STREAM( _nh_camera.getNamespace());
    _syncronized_camera_sub.reset(new rgbd_utils::RGBD_Subscriber("/camera/rgb/camera_info",
                                                                  "/camera/rgb/image_raw",
                                                                  "/camera/depth_registered/camera_info",
                                                                  "/camera/depth_registered/image_raw",
                                                                  _nh_camera));
    _camera_topics_sub = _nh_camera.subscribe<sensor_msgs::Image>("/camera/depth_registered/image_raw", 1, &CAMERA_kinect_openni::camera_topics_start_publishing_cb, this);
    std::string camera_file_path;
    _nh_camera.getParam("/camera_file_path", camera_file_path);
    _global_parameters.get_camera_character().readFromXMLFile(camera_file_path);
    _camera_spinner.reset(new ros::AsyncSpinner(1));
    _camera_spinner->start();
}

void BAXTER::init(){
    std::string baxter_arm = std::string(_global_parameters.get_parameters()["baxter_arm"]);
    if(strcmp(baxter_arm.c_str(), "left") == 0)
        _eef_state_sub.reset(new ros::Subscriber(_baxter_nh.subscribe("/robot/limb/left/endpoint_state", 10, &BAXTER::eef_Callback, this)));
    else if(strcmp(baxter_arm.c_str(), "right") == 0)
        _eef_state_sub.reset(new ros::Subscriber(_baxter_nh.subscribe("/robot/limb/right/endpoint_state", 10, &BAXTER::eef_Callback, this)));
    _baxter_spinner.reset(new ros::AsyncSpinner(1));
    _baxter_spinner->start();
}

void CRUSTCRAWLER::init(){
    _eef_state_sub.reset(new ros::Subscriber(_crustcrawler_nh.subscribe("/crustcrawler/endpoint_state", 10, &CRUSTCRAWLER::eef_Callback, this)));
    _crustcrawler_spinner.reset(new ros::AsyncSpinner(1));
    _crustcrawler_spinner->start();
}

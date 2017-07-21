#include <visual_functionalities/visual_functionalities.hpp>

using namespace visual_functionalities;

void VISUAL_FUNCTIONALITIES::init(){
    ROS_INFO("VISUAL_FUNCTIONALITIES: initializing ......");
    _visual_functionalities_service.reset(new ros::ServiceServer(
                                              _nh_visual_functionalities.advertiseService("object_detection_by_qr_code",
                                                                                          &VISUAL_FUNCTIONALITIES::get_object_position_cb,
                                                                                          this)));
    _object_qr_position_pub.reset(new ros::Publisher(
                                      _nh_visual_functionalities.advertise<visual_functionalities::object_qr_position>("object_qr_position", true)));
    _nh_visual_functionalities.getParam("/visual_functionalities_parameters", _global_parameters.get_parameters());
    _global_parameters.set_marker_size(std::stof(_global_parameters.get_parameters()["marker_size"]));
    _global_parameters.set_child_frame(std::string(_global_parameters.get_parameters()["child_frame"]));
    _global_parameters.set_parent_frame(std::string(_global_parameters.get_parameters()["parent_frame"]));

    //set the robot
    std::string camera_name = std::string(_global_parameters.get_parameters()["camera"]);
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

void VISUAL_FUNCTIONALITIES::show_image(){
    _global_parameters.get_rgb_msg().reset(new sensor_msgs::Image(_camera->_syncronized_camera_sub->get_rgb()));
    _global_parameters.get_depth_msg().reset(new sensor_msgs::Image(_camera->_syncronized_camera_sub->get_depth()));
    _global_parameters.get_camera_info_msg().reset(new sensor_msgs::CameraInfo(_camera->_syncronized_camera_sub->get_rgb_info()));
    try{
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(_global_parameters.get_rgb_msg(), sensor_msgs::image_encodings::BGR8);
        _global_parameters.set_cvt(cv_ptr);
        _global_parameters.set_raw_original_picture(_global_parameters.get_cvt()->image);
        _global_parameters.get_aruco_marker_detector().setDictionary("ARUCO");
        _global_parameters.get_aruco_marker_detector().detect(_global_parameters.get_raw_original_picture(),
                                                              _global_parameters.get_markers(),
                                                              _global_parameters.get_camera_character(),
                                                              0.1);
        _global_parameters.get_marker_centers().resize(_global_parameters.get_markers().size());

        //        ROS_WARN_STREAM("VISUAL_FUNCTIONALITIES: I found markers, number of them is: " << _global_parameters.get_markers().size());
        //        if(!_global_parameters.get_markers().empty())
        //            ROS_WARN_STREAM("VISUAL_FUNCTIONALITIES: And their IDs are: ");

        for(size_t i = 0; i < _global_parameters.get_markers().size(); i++){
            //            ROS_WARN_STREAM("VISUAL_FUNCTIONALITIES: For marker number: " << i << " The ID is: " << _global_parameters.get_markers()[i].id);
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
            double x = _global_parameters.get_marker_centers()[i](0), y = _global_parameters.get_marker_centers()[i](1);
            pcl::PointXYZRGBA pt_marker = input_cloud->at(std::round(x) + std::round(y) * input_cloud->width);
            if(pt_marker.x == pt_marker.x && pt_marker.y == pt_marker.y && pt_marker.z == pt_marker.z){
                geometry_msgs::PointStamped object_position;
                object_position.header.stamp = ros::Time::now();
                object_position.point.x = pt_marker.x;
                object_position.point.y = pt_marker.y;
                object_position.point.z = pt_marker.z;
                _global_parameters.set_object_position(_global_parameters.get_markers()[i].id, object_position);
                visual_functionalities::object_qr_position object_qr_position_topic;
                object_qr_position_topic.qr_id.data = _global_parameters.get_markers()[i].id;
                object_qr_position_topic.object_qr_position = object_position;
                _object_qr_position_pub->publish(object_qr_position_topic);
                _global_parameters.get_objects_positions_map().find(_global_parameters.get_markers()[i].id)->second = object_position;

            }
            //            else{
            //                ROS_WARN_STREAM("VISUAL_FUNCTIONALITIES: The marker position is nan: " << pt_marker.x << ", " << pt_marker.y << ", " << pt_marker.z);
            //            }
        }
    }
    catch(...){
        //        ROS_ERROR("Something went wrong !!!");
        return;
    }

    cv::namedWindow("ShowMarker",CV_WINDOW_AUTOSIZE);
    if(!_global_parameters.get_raw_original_picture().empty()){
        cv::imshow("ShowMarker", _global_parameters.get_raw_original_picture());
        cv::waitKey(1);
    }
}


bool VISUAL_FUNCTIONALITIES::get_object_position_cb(visual_functionalities::object_detection_by_qr_code::Request &req,
                                                    visual_functionalities::object_detection_by_qr_code::Response &res){
    ROS_INFO("VISUAL_FUNCTIONALITIES: at get object position, marker size is: %d",  _global_parameters.get_markers().size());
    bool marker_exist = false;
    for(auto& i: _global_parameters.get_objects_positions_map()){
        if(req.object_index == i.first){
            for(size_t m = 0; m < _global_parameters.get_markers().size(); m++){
                if(_global_parameters.get_markers()[m].id == req.object_index)
                    marker_exist = true;
            }
            if(marker_exist){
                Eigen::Vector3d position_camera_frame, position_robot_frame;
                position_camera_frame << i.second.point.x, i.second.point.y, i.second.point.z;
                lib_recording_functions::convert_object_position_to_robot_base(_global_parameters,
                                                                               position_camera_frame,
                                                                               position_robot_frame);
                res.model_state = {position_robot_frame(0), position_robot_frame(1), position_robot_frame(2)};
                return true;
            }
            else
                return false;
        }
    }
    return false;
}

void VISUAL_FUNCTIONALITIES::convert_vector_object_position_robot_frame(
        std::vector<std::vector<double> > &object_position_camera_frame_vector,
        std::vector<std::vector<double> > &object_position_robot_frame_vector){
    lib_recording_functions::convert_whole_object_positions_vector(_global_parameters,
                                                                   object_position_camera_frame_vector,
                                                                   object_position_robot_frame_vector);
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
                                                                  "/camera/rgb/image_color",
                                                                  "/camera/depth_registered/sw_registered/camera_info",
                                                                  "/camera/depth_registered/sw_registered/image_rect",
                                                                  _nh_camera));
    //    _camera_topics_rgb_info_sub = _nh_camera.subscribe<sensor_msgs::CameraInfo>("/camera/rgb/camera_info", 1, &CAMERA_kinect_freenect::camera_topics_rgb_info_cb, this);
    //    _camera_topics_rgb_image_sub = _nh_camera.subscribe<sensor_msgs::Image>("/camera/rgb/image_color", 1, &CAMERA_kinect_freenect::camera_topics_rgb_image_cb, this);
    //    _camera_topics_depth_info_sub = _nh_camera.subscribe<sensor_msgs::CameraInfo>("/camera/depth_registered/sw_registered/camera_info", 1, &CAMERA_kinect_freenect::camera_topics_depth_info_cb, this);
    _camera_topics_sub = _nh_camera.subscribe<sensor_msgs::Image>("/camera/depth_registered/sw_registered/image_rect", 1, &CAMERA_kinect_freenect::camera_topics_start_publishing_cb, this);
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

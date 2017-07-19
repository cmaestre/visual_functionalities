#ifndef _BAXTER_MOVER_HPP
#define _BAXTER_MOVER_HPP

#include "lib_recording.hpp"

namespace visual_functionalities {
Visual_values _global_parameters;
class CAMERA;
class VISUAL_FUNCTIONALITIES;

class CAMERA{
public:
    typedef std::shared_ptr<CAMERA> Ptr;
    typedef const std::shared_ptr<CAMERA> ConstPtr;

    CAMERA(){}
    virtual void init() = 0;

    friend class VISUAL_FUNCTIONALITIES;
protected:
    ros::NodeHandle _nh_camera;
    std::unique_ptr<ros::AsyncSpinner> _camera_spinner;
    ros::Subscriber _camera_topics_sub;
    std::shared_ptr<rgbd_utils::RGBD_Subscriber> _syncronized_camera_sub;
};

class CAMERA_kinect_freenect : public CAMERA{
public:
    typedef std::shared_ptr<CAMERA_kinect_freenect> Ptr;
    typedef const std::shared_ptr<CAMERA_kinect_freenect> ConstPtr;

    CAMERA_kinect_freenect(){
    }

    void init() override;
    void camera_topics_start_publishing_cb(const sensor_msgs::Image::ConstPtr& depth_msgs){
        _global_parameters.set_camera_topics_status(true);
    }
};

class CAMERA_kinect_openni : public CAMERA{
public:
    typedef std::shared_ptr<CAMERA_kinect_openni> Ptr;
    typedef const std::shared_ptr<CAMERA_kinect_openni> ConstPtr;

    CAMERA_kinect_openni(){
    }

    void init() override;
    void camera_topics_start_publishing_cb(const sensor_msgs::Image::ConstPtr& depth_msgs){
        _global_parameters.set_camera_topics_status(true);
    }
};

class CAMERA_kinect_v2 : public CAMERA{
public:
    typedef std::shared_ptr<CAMERA_kinect_v2> Ptr;
    typedef const std::shared_ptr<CAMERA_kinect_v2> ConstPtr;

    CAMERA_kinect_v2(){
    }

    void init() override;
    void camera_topics_start_publishing_cb(const sensor_msgs::Image::ConstPtr& depth_msgs){
        _global_parameters.set_camera_topics_status(true);
    }
};


class VISUAL_FUNCTIONALITIES{
public:
    typedef std::shared_ptr<VISUAL_FUNCTIONALITIES> Ptr;
    typedef const std::shared_ptr<VISUAL_FUNCTIONALITIES> ConstPtr;

    VISUAL_FUNCTIONALITIES(){
        init();
    }

    void init();

    bool get_object_position_cb(visual_functionalities::object_detection_by_qr_code::Request &req,
                                visual_functionalities::object_detection_by_qr_code::Response &res);

    void convert_vector_object_position_robot_frame(std::vector<std::vector<double>>& object_position_camera_frame_vector,
                                                    std::vector<std::vector<double>>& object_position_robot_frame_vector);

    void show_image();

    Visual_values& get_global_parameters(){
        return _global_parameters;
    }
private:
    ros::NodeHandle _nh_visual_functionalities;
    std::unique_ptr<ros::AsyncSpinner> _visual_functionalities_spinner;
    std::unique_ptr<ros::ServiceServer> _visual_functionalities_service;
    std::unique_ptr<ros::Publisher> _object_qr_position_pub;
    CAMERA::Ptr _camera;
};

}

#endif //_BAXTER_MOVER_HPP

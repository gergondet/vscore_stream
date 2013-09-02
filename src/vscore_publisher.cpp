#include "vscore_publisher.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <sstream>

struct VSCorePublisherImpl
{
public:
    VSCorePublisherImpl() : n(), it(n) 
    {
    }

    void CreatePublisher(visionsystem::Camera * cam)
    {
        cams.push_back(cam);
        std::stringstream ss;
        ss << cam->get_name() << "/image";
        pubs[cam] = it.advertise(ss.str().c_str(), 1);
    }

    void PublishImage(visionsystem::Camera * cam, vision::Image<uint32_t, vision::RGB> & img)
    {
        sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
        msg->header.seq = cam->get_frame();
        msg->header.stamp = ros::Time::now();
        msg->height = cam->get_size().y;
        msg->width = cam->get_size().x;
        msg->encoding = "bgr8";
        msg->step = msg->width*3;
        msg->data.resize(msg->step*msg->height);
        for(unsigned int i = 0; i < img.pixels; ++i)
        {
            msg->data[3*i] = ((unsigned char*)(img.raw_data))[4*i+2];
            msg->data[3*i+1] = ((unsigned char *)(img.raw_data))[4*i+1];
            msg->data[3*i+2] = ((unsigned char *)(img.raw_data))[4*i];
        }
        pubs[cam].publish(msg);
    }

public:
    ros::NodeHandle n;
    image_transport::ImageTransport it;
    std::map< visionsystem::Camera *, image_transport::Publisher> pubs;
    std::vector<visionsystem::Camera *> cams;
};

VSCorePublisher::VSCorePublisher( visionsystem::VisionSystem* vs, std::string sandbox )
: visionsystem::Plugin(vs, "vscore_publisher", sandbox)
{
    int argc = 0; char * argv[] = {};
    ros::init(argc, argv, "vscore_publisher");
    impl = new VSCorePublisherImpl();
}

VSCorePublisher::~VSCorePublisher()
{
    delete impl;
}

bool VSCorePublisher::pre_fct()
{
    std::vector<visionsystem::Camera *> cams = get_all_cameras();

    for(size_t i = 0; i < cams.size(); ++i)
    {
        if(cams[i]->is_active())
        {
            register_to_cam< vision::Image<uint32_t, vision::RGB> >(cams[i], 3);
            impl->CreatePublisher(cams[i]);
        }
    }

    return true;
}

void VSCorePublisher::preloop_fct()
{
}

void VSCorePublisher::loop_fct()
{
    for(size_t i = 0; i < impl->cams.size(); ++i)
    {
        vision::Image<uint32_t, vision::RGB> * img = dequeue_image< vision::Image<uint32_t, vision::RGB> >( impl->cams[i] ) ;
        impl->PublishImage(impl->cams[i], *img);
        enqueue_image< vision::Image<uint32_t, vision::RGB> >(impl->cams[i], img);
    }
}

bool VSCorePublisher::post_fct()
{
    for(size_t i = 0; i < impl->cams.size(); ++i)
    {
        unregister_to_cam< vision::Image<uint32_t, vision::RGB> >(impl->cams[i]);
    }
    return true;
}

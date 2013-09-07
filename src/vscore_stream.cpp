#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <boost/thread.hpp>
#include <visionsystem/vs_controllers/socket/camerasocket.h>

int main(int argc, char * argv[])
{
    if(argc < 2)
    {
        std::cerr << "Should provide at least one configuration file" << std::endl;
        return 1;
    }
    std::string config = argv[1];

    ros::init(argc, argv, "vscore_stream");
    std::cout << argc << std::endl;

    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::CameraPublisher pub = it.advertiseCamera("vscore/image", 1);
    ros::Rate rate(60);

    sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo);
    info->height = 480;
    info->width = 640;
    info->distortion_model = "plumb_bob";
    info->D.resize(5);
    info->D[0] = 0.031828;
    info->D[1] = -0.106567;
    info->D[2] = -0.001167;
    info->D[3] = 0.002602;
    info->D[4] = 0;
    info->K[0] = 538.047049;
    info->K[2] = 319.509874;
    info->K[4] = 538.226039;
    info->K[5] = 239.332725;
    info->K[8] = 1;
    info->P[0] = 538.047049;
    info->P[2] = 319.509874;
    info->P[5] = 538.226039;
    info->P[6] = 239.332725;
    info->P[10] = 1;

    boost::asio::io_service io;
    visionsystem::CameraSocket * cam = new visionsystem::CameraSocket(io);
    cam->read_config_file( config.c_str() );
    cam->start_cam();
    boost::thread * io_service_th = new boost::thread(boost::bind(&boost::asio::io_service::run, &io));

    while(ros::ok())
    {
        if(cam->has_data())
        {
            sensor_msgs::ImagePtr img(new sensor_msgs::Image);
            info->header.seq++;
            info->header.stamp = ros::Time::now();
            img->header = info->header;
            img->height = 480;
            img->width = 640;
            img->encoding = "bgr8";
            img->step = img->width*3;
            img->data.resize(img->step*img->height);
            for(unsigned int i = 0; i < img->height*img->width; ++i)
            {
                img->data[3*i] = cam->get_data()[4*i+2];
                img->data[3*i+1] = cam->get_data()[4*i+1];
                img->data[3*i+2] = cam->get_data()[4*i];
            }
            pub.publish(img, info);
        }
        rate.sleep();
    }
    io.stop();
    io_service_th->join();
    delete io_service_th;
    delete cam;

    return 0;
}

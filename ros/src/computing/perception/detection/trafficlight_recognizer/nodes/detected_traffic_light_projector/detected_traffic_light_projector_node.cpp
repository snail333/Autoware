// headers for ros
#include <ros/ros.h>

// headers in this package
#include <detected_traffic_light_projector/detected_traffic_light_projector.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "detected_traffic_light_projector_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    DetectedTrafficLightProjector projector(nh,pnh);
    return 0;
}
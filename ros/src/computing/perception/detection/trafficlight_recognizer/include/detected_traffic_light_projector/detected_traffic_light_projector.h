#ifndef DETECTED_TRAFFIC_LIGHT_PROJECTOR_H_INCLUDED
#define DETECTED_TRAFFIC_LIGHT_PROJECTOR_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//heades in Autoware
#include <libvectormap/vector_map.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/Signals.h>

//headers in STL
#include <map>

class DetectedTrafficLightProjector
{
public:
    DetectedTrafficLightProjector(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~DetectedTrafficLightProjector();
private:
    ros::NodeHandle pnh_;
    ros::NodeHandle nh_;
    void detectedObjectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr msg);
    ros::Subscriber detected_objects_sub_;
    ros::Publisher signals_pub_;
    VectorMap vmap;
    std::map<std::string,ros::Subscriber> vector_map_subs_;
    std::vector<geometry_msgs::PoseStamped> getSignalPose();
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string camera_frame_;
    std::string map_frame_;
    geometry_msgs::Point convertPointToGeomPoint(const Point& point);
    geometry_msgs::Quaternion convertVectorToGeomQuaternion(const Vector& vector);
    inline double convertDegreeToRadian(double degree);
};

#endif  //DETECTED_TRAFFIC_LIGHT_PROJECTOR_H_INCLUDED
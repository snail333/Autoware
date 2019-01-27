// headers in this package
#include <detected_traffic_light_projector/detected_traffic_light_projector.h>

DetectedTrafficLightProjector::DetectedTrafficLightProjector(ros::NodeHandle nh,ros::NodeHandle pnh) : tf_listener_(tf_buffer_)
{
    nh_ = nh;
    pnh_ = pnh;
    signals_pub_ = nh_.advertise<autoware_msgs::Signals>("/roi_signal",10);
    pnh_.param<std::string>("camera_frame", camera_frame_, "camera");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    // initializa vector map topic
    vector_map_subs_["point"] = nh_.subscribe("/vector_map_info/point",10,&VectorMap::load_points,&vmap);
    vector_map_subs_["line"] = nh_.subscribe("/vector_map_info/line",10,&VectorMap::load_lines,&vmap);
    vector_map_subs_["lane"] = nh_.subscribe("/vector_map_info/lane",10,&VectorMap::load_lanes,&vmap);
    vector_map_subs_["vector"] = nh_.subscribe("/vector_map_info/vector",10,&VectorMap::load_vectors,&vmap);
    vector_map_subs_["signal"] = nh_.subscribe("/vector_map_info/signal",10,&VectorMap::load_signals,&vmap);
    vector_map_subs_["whiteline"] = nh_.subscribe("/vector_map_info/whiteline",10,&VectorMap::load_whitelines,&vmap);
    vector_map_subs_["dtlane"] = nh_.subscribe("/vector_map_info/dtlane",10,&VectorMap::load_dtlanes,&vmap);
}

DetectedTrafficLightProjector::~DetectedTrafficLightProjector()
{

}

double DetectedTrafficLightProjector::convertDegreeToRadian(double degree)
{
    return degree * M_PI / 180;
}

geometry_msgs::Quaternion DetectedTrafficLightProjector::convertVectorToGeomQuaternion(const Vector& vector)
{
    double pitch = convertDegreeToRadian(vector.vang - 90); // convert vertical angle to pitch
    double yaw = convertDegreeToRadian(-vector.hang + 90); // convert horizontal angle to yaw
    return tf::createQuaternionMsgFromRollPitchYaw(0, pitch, yaw);
}

geometry_msgs::Point DetectedTrafficLightProjector::convertPointToGeomPoint(const Point& point)
{
    geometry_msgs::Point geom_point;
    // NOTE: Autwoare use Japan Plane Rectangular Coordinate System.
    // Therefore we swap x and y axis.
    geom_point.x = point.ly;
    geom_point.y = point.bx;
    geom_point.z = point.h;
    return geom_point;
}

std::vector<geometry_msgs::PoseStamped> DetectedTrafficLightProjector::getSignalPose()
{
    std::vector<geometry_msgs::PoseStamped> signal_pose;
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer_.lookupTransform(camera_frame_, map_frame_, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return signal_pose;
    }
    for(auto signal_itr = vmap.signals.begin(); signal_itr != vmap.signals.end(); signal_itr++)
    {
        Vector v = vmap.vectors[signal_itr->second.vid];
        Point p = vmap.points[v.pid];
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = map_frame_;
        pose.pose.position = convertPointToGeomPoint(p);
        pose.pose.orientation = convertVectorToGeomQuaternion(v);
        tf2::doTransform(pose,pose,transform_stamped);
        signal_pose.push_back(pose);
    }
    return signal_pose;
}

void DetectedTrafficLightProjector::detectedObjectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr msg)
{
    if (vmap.points.empty() || vmap.lines.empty() || vmap.whitelines.empty() || vmap.lanes.empty() || vmap.dtlanes.empty() || vmap.vectors.empty() || vmap.signals.empty())
    {
        return;
    }
    std::vector<autoware_msgs::DetectedObject> traffic_light_objects;
    for(auto object_itr = msg->objects.begin(); object_itr != msg->objects.end(); object_itr++)
    {
        if(object_itr->label == "traffic light")
        {
            traffic_light_objects.push_back(*object_itr);
        }
    }
    for(auto traffic_light_itr = traffic_light_objects.begin(); traffic_light_itr != traffic_light_objects.end(); traffic_light_itr++)
    {

    }
    return;
}
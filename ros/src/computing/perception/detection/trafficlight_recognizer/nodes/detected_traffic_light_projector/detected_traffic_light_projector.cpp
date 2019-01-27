// headers in this package
#include <detected_traffic_light_projector/detected_traffic_light_projector.h>

DetectedTrafficLightProjector::DetectedTrafficLightProjector(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    signals_pub_ = nh_.advertise<autoware_msgs::Signals>("/roi_signal",10);
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
    return;
}
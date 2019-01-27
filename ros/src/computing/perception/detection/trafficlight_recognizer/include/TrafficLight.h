#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H

/* Extra includes */
#include "autoware_msgs/Signals.h"
#include "autoware_msgs/TrafficLight.h"

#define MAIN_WINDOW_NAME "Main"
#define SETTINGS_WINDOW_NAME "Settings"

#define TLR_GREEN_SIGNAL_STR "green signal"
#define TLR_RED_SIGNAL_STR "red signal"
#define TLR_UNKNOWN_SIGNAL_STR ""
constexpr int TRAFFIC_LIGHT_RED = autoware_msgs::TrafficLight::RED;
constexpr int TRAFFIC_LIGHT_GREEN = autoware_msgs::TrafficLight::GREEN;
constexpr int TRAFFIC_LIGHT_UNKNOWN = autoware_msgs::TrafficLight::UNKNOWN;

#endif

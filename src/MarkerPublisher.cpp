#include "cam_exploration/MarkerPublisher.h"
#include "cam_exploration/MapServer.h"

#include "tf/transform_datatypes.h"


/**
 * @file MarkerPublisher.cpp
 * @brief Implementation of the MarkerPublisher.h file
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-21
 */

namespace cam_exploration {
    
// namespace cam_exploration--------------------------------
// 		Class pub
//------------------------------------------------------------
pub::pub(const char* name, const char* topic, ros::NodeHandle n, int type)
{
    name_ = name;
    topic_ = topic;
    n_ = n;

    visualization_msgs::Marker m;
    m.header.frame_id = "/map";
    m.type = type;
    m.scale.x = m.scale.y = m.scale.z = 0.07;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0);
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    m_ = m;

    initialise();
}

void pub::initialise()
{
    p_ = n_.advertise<visualization_msgs::Marker>(topic_, 5);
}

void pub::publish( geometry_msgs::Point p, int yaw) const
{
    geometry_msgs::Quaternion q;
    geometry_msgs::Pose pos;
    tf::quaternionTFToMsg (tf::createQuaternionFromRPY(0.0,0.0,yaw), q);
    pos.position = p;
    pos.orientation = q;
    publish_(pos);
}


void pub::publish_( geometry_msgs::Pose p ) const
{
    visualization_msgs::Marker m = m_;
    m.header.stamp = ros::Time::now();
    m.pose = p;
    p_.publish(m);
}
void pub::publish_( const std::vector<geometry_msgs::Point>& points) const
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;

    std::vector<std_msgs::ColorRGBA> c(points.size(), color);

    visualization_msgs::Marker m = m_;

    m.header.stamp = ros::Time::now();
    m.ns = name_;

    if (m.type == visualization_msgs::Marker::SPHERE)
	m.type = visualization_msgs::Marker::POINTS;
    m.pose.position.x = m.pose.position.y = 0.0; m.pose.position.z = 0;
    m.pose.orientation.x = m.pose.orientation.y = m.pose.orientation.z = 0; m.pose.orientation.w = 1;
    m.points = points;
    //m.color.g = m.color.a = 0;
    m.colors = c;

    p_.publish(m);
}


//------------------------------------------------------------
// 		Class MarkerPublisher
//------------------------------------------------------------
std::vector<pub> MarkerPublisher::pubs;

MarkerPublisher::MarkerPublisher()
{
    initialise();
}

MarkerPublisher::MarkerPublisher(ros::NodeHandle n_in) : n(n_in)
{
    initialise();
}

void MarkerPublisher::initialise()
{
    marker.header.stamp = ros::Time();
    marker.ns = "MarkerPublisher";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
}


void MarkerPublisher::setNh(ros::NodeHandle n_in)
{
    n = n_in;
}


void MarkerPublisher::publish(geometry_msgs::Point position, const char* topic, const char* frame)
{
    visualization_msgs::Marker m = marker;
    m.pose.position = position;
    ros::Publisher pub = n.advertise<visualization_msgs::Marker>(topic, 10);
    m.header.frame_id = frame;

    pub.publish(m);
}


void MarkerPublisher::add(const char* name, const char* topic, int type)
{
    pub p(name, topic, n, type);
    pubs.push_back(p);
    ROS_INFO("MarkerPublisher: Advertised topic %s", topic);
}


} // namespace cam_exploration



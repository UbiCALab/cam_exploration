#include <cam_exploration/goalSelector.h>
#include <cam_exploration/RobotMotion.h>
#include <cam_exploration/MapServer.h>


/** 
 * @file goalSelector.cpp
 * @brief Implementation of goalSelector.h file
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-21
 */


namespace cam_exploration {
namespace strategy {


void goalSelector::init()
{
    ros::NodeHandle nh("~/goal_selector");

    nh.param<double>("distance_to_goal", distance_to_goal, 1);

    MarkerPublisher markers;
    markers.add("goal_padding", "goal_padding", visualization_msgs::Marker::LINE_STRIP);
}


geometry_msgs::Quaternion goalSelector::quat(double yaw) const
{
    geometry_msgs::Quaternion out;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), out);
    return out;
}

bool goalSelector::aimAt(const geometry_msgs::Point& frontier_point,
					geometry_msgs::Pose& goal_out) const
{
    MarkerPublisher markers;

    geometry_msgs::Point p_goal, r_pose = RobotMotion::position();
    geometry_msgs::Pose goal;
    std::vector<geometry_msgs::Point> points;

    double angle = atan2(r_pose.y - frontier_point.y, r_pose.x - frontier_point.x);

    float divider = 6;
    unsigned int steps = 0, turns = 0;
    angle += (M_PI - M_PI/divider);

    do{
    	points.clear();
    	if (steps / divider > 2){
    	    steps = 0;
    	    divider /= 2;
    	    ++turns;
                //ROS_INFO("Turn: %d", turns);
	}
	angle += M_PI / divider;

	p_goal.x = frontier_point.x - cos(angle) * distance_to_goal;
	p_goal.y = frontier_point.y - sin(angle) * distance_to_goal;
	p_goal.z = 0;

	goal.position = p_goal;
	goal.orientation = quat(angle);

	points.push_back(p_goal);
	points.push_back(frontier_point);

	markers.publish("goal_padding", points);
	steps++;
    }
    while(!hasGoalAtSight(points) && turns < 2);

    markers.publish("goal_padding", points);

    goal_out = goal;

    return true;
}


bool goalSelector::hasGoalAtSight(const std::vector<geometry_msgs::Point>& points) const
{
    double step_size = 0.1, delta = 0;
    int iterations = floor(distance_to_goal / step_size);
    bool collide = false;
    MapServer map_server;
    for (int i = 0; i < iterations && !collide; ++i) {
    	delta = i / iterations;
	geometry_msgs::Point p;
	p.x = points[0].x * delta + points[1].x * (1-delta);
	p.y = points[0].y * delta + points[1].y * (1-delta);
	p.z = 0;
    	if (!map_server.isFree(p)){
	    collide = true;
	}
    }

    // TODO Correct "collision detection"
    //const char* x = !collide ? "true" : "false";
    //ROS_INFO("Has goal at sight? %s", x);
    return !collide;
}


geometry_msgs::Pose midPoint::decideGoal(const frontier& f) const
{
    geometry_msgs::Point f_point = f.free_center_point;
    geometry_msgs::Pose goal;
    aimAt(f_point, goal);
    return goal;
}




} // namespace strategy
} // namespace cam_exploration



/*
 * MIT License
 *
 * Copyright (c) 2016 Jordi Soler Busquets
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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
    std_msgs::ColorRGBA color;
    color.r = 0.2; color.g = 0.2; color.b = 0.6; color.a = 1.0;

    markers.add("goal_padding", "goal_padding");
    //markers.add("goal_padding", "goal_padding", visualization_msgs::Marker::LINE_STRIP);
    markers.setProperty("goal_padding", color);
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
    //MarkerPublisher markers;
    RobotMotion robot;

    geometry_msgs::Point p_goal, r_pose = RobotMotion::position();
    geometry_msgs::Pose goal;
    std::vector<geometry_msgs::Point> points;

    robot.setFrontierPoint(frontier_point);

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

	//markers.publish("goal_padding", points);
	steps++;
    }
    while(!hasGoalAtSight(points) && turns < 2);

    //markers.publish("goal_padding", points);

    goal_out = goal;

    return turns < 2;
}


bool goalSelector::hasGoalAtSight(const std::vector<geometry_msgs::Point>& points) const
{
    MarkerPublisher markers;

    double step_size = 0.05, delta = 0;
    double iterations = floor(distance_to_goal / step_size);
    bool collide = false;
    MapServer map_server;
    std::vector<geometry_msgs::Point> samples;
    for (int i = 0; i < iterations && !collide; ++i) {
    	delta = i / iterations;
	geometry_msgs::Point p;
	p.x = points[0].x * delta + points[1].x * (1-delta);
	p.y = points[0].y * delta + points[1].y * (1-delta);
	p.z = 0;

	samples.push_back(p);
    	if (!map_server.isFree(p)){
	    collide = true;
	}
    }

    markers.publish("goal_padding", samples);

    // TODO Correct "collision detection"
    //const char* x = !collide ? "true" : "false";
    //ROS_INFO("Has goal at sight? %s", x);
    return !collide;
}


bool midPoint::decideGoal(const frontier& f, geometry_msgs::Pose& goal) const
{
    geometry_msgs::Point f_point = f.free_center_point;
    bool hasBeenFound = aimAt(f_point, goal);

    return hasBeenFound;
}




} // namespace strategy
} // namespace cam_exploration


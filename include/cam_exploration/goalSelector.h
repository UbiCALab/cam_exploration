#ifndef CAM_EXPLORATION_GOAL_SELECTOR
#define CAM_EXPLORATION_GOAL_SELECTOR

/**
 * @file goalSelector.h
 * @brief This file provides a set of objects which aim to choose a proper goal to sent to move_base given a frontier
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-19
 */

#include <cam_exploration/frontier.h>
#include <tf/transform_datatypes.h>

namespace cam_exploration{
namespace strategy{

/**
 * @brief Base class for goal decision objects
 */
class goalSelector
{
public:
    /**
     * @brief Get the goal of a certain frontier
     *
     * @param frontier_in Frontier to be evaluated
     *
     * @return Goal to which the robot should move
     */
    virtual geometry_msgs::Pose decideGoal(const frontier& frontier_in) const = 0;
    /**
     * @brief Get name of the goal decision object
     *
     * @return Name of the goal decision object
     */
    virtual const char* name() const = 0;

    /**
     * @brief Get a quaternion representing an angle in the XY plane
     *
     * @param yaw Yaw Euler angle encoding the desired angle in the plane
     *
     * @return The corresponding quaternion
     */
    geometry_msgs::Quaternion quat(double yaw) const;

    /**
     * @brief Find a proper goal given a frontier point
     *
     *	As the ground projection of the RGBA cameras is used for exploration, the direct use of a frontier point can lead to the robot getting stuck as it usually can't erese the frontier wich is "down its feet". Hence, a \p goal is fond such that the robot can see the point \p frontier_point
     *
     * @param frontier_point Point of interest in the goal frontier
     * @param goal Goal where the robot should go
     *
     * @return True if a proper goal could be found, false otherwise
     */
    bool aimAt(const geometry_msgs::Point& frontier_point, geometry_msgs::Pose& goal) const;

    /** 
     * @brief Get parameters from the Parameter Server
     */
    void init();
private:

    /** 
     * @brief The distance between \p frontier_point and \p goal of goalSelector::aimAt function
     *
     * This distance should be slightly higher than the distance between the robot base and the nearest ground projected RGBA camera point
     */
    double distance_to_goal;
    /** 
     * @brief Check wether a set of points are valid points.
     *
     * This function is thought to be used to check the path from \p frontier_point to \p goal of goalSelector::aimAt function 
     * 
     * @param points Set of points for which collision is to be checked
     * 
     * @return True if there is not a non-free point, false otherwise
     */
    bool hasGoalAtSight(const std::vector<geometry_msgs::Point>& points) const;
};


/** 
 * @brief Goal selector which looks for the frontier's middle point
 */
class midPoint : public goalSelector
{
public:
    /** 
     * @brief Parameterless constructor
     */
    midPoint(){ init(); }
    /** 
     * @brief Select the middle point of the frontier \p frontier_in and returh a goal aiming at it
     * 
     * @param frontier_in Incoming frontier
     * 
     * @return Goal pose
     */
    geometry_msgs::Pose decideGoal(const frontier& frontier_in) const;
    const char* name() const { return "Middle Point";}
};




} /* cam_exploration */
} /* strategy */



#endif  // CAM_EXPLORATION_GOAL_SELECTOR


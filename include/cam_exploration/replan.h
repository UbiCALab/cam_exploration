
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


#ifndef CAM_EXPLORATION_REPLAN_H
#define CAM_EXPLORATION_REPLAN_H

/**
 * @file replan.h
 * @brief Stores all the functionallities related with replaning a new goal
 * @author Jordi Soler
 * @version 1
 * @date 2016-04-21
 */



#include <cam_exploration/RobotMotion.h>

namespace cam_exploration {


/**
 * @brief Stores all the functionalities related with replanning a new goal
 */
namespace replan {


/**
 * @brief Cause for replaning.
 *
 * This is a base interface class for many implementations each of which should follow a different replaning criteria
 */
class replaningCause
{
    public:
    /**
     * @brief Decides wether the robot should replan a new goal
     *
     * @return True if a new goal should be replaned
     */
    virtual bool replan() = 0;
    /**
     * @brief Identifies the replaning cause
     *
     * @return Identifier of the replaning cause
     */
    virtual const char* name() = 0;
};


/**
 * @brief Cause for replanning
 *
 * Takes the decision considering the robot motion status. It does replan when the robot is not moving, that is, at startup or when the robot couldn't reach / has reached the current goal.
 */
class NotMoving : public replaningCause
{
    public:
    bool replan() { return !RobotMotion::isMoving();}
    const char * name() { return "Not moving";}

};


/**
 * @brief Cause for replanning
 *
 * Takes into consideration the time spent near the goal. If the robot spents too much time near the goal and it is roughly aiming in the same direction as the goal, a new goal should be replaned.
 */
class TooMuchTimeNearGoal : public replaningCause
{
public:
    TooMuchTimeNearGoal(std::map<std::string, std::string> params);

    bool replan();
    const char * name() { return "Spent too much time near the goal";}

private:
    ros::Duration time_th_;		///< Maximum time allowed for the robot to be near the goal (if the orientation condition is met)
    double distance_th_;		///< Maximum distance between the robot and the goal to consider the robot to be near the goal
    double orientation_th;		///< Tolerance angle to consider the robot to be aiming at the same direction as the goal.

    ros::Time prev;			///< Last time when the robot was not near the goal
    bool near_goal;			///< True if the robot is near the goal

    /**
     * @brief Checks robot slignment with the goal direction
     *
     * @return True if the robot is aligned with the goal direction
     */
    bool isOriented();
    /**
     * @brief Checks the proximity to the robot with the goal
     *
     * @return True if the robot is near the goal
     */
    bool isNearGoal();
};


/**
 * @brief Cause for replanning
 *
 * Takes into consideration the proximity of the goal with the current frontiers. The robot should replan if the goal is no longer near to any frontier.
 */
class IsolatedGoal : public replaningCause
{
public:
    IsolatedGoal(std::map<std::string, std::string> params);

    bool replan();
    const char * name() { return "The goal is no longer close to any frontier";}

private:
    unsigned int depth_;			///< Degree of proximity to consider the goal close to a frontier cell. @see cam_exploration::MapServer::getNeighbours(int,unsigned int)

    /**
     * @brief Checks the proximity of the goal with any frontier cell
     *
     * @return True if the goal is near a frontier
     */
    bool isNearFrontier();
};




// Replaning function
/**
 * @brief Object that decides if a new goal should be replanned according to different criteria.
 */
class Replaner
{
public:
    Replaner(){ verbosity = 1;}

    /**
     * @brief Decides replaning
     *
     * @return True if a new goal should be replanned
     */
    bool replan();

    /**
     * @copydoc addCause(std::string)
     *
     * @param parameters Parameters of the replaning cause
     */
    void addCause(std::string name, std::map<std::string, std::string> parameters);
    /**
     * @brief Add a new replanning cause
     *
     * @param name Replaning cause identifier
     */
    void addCause(std::string name) {
	std::map<std::string, std::string> maps;
    	addCause(name, maps);
    }

private:
    /**
     * @brief Collection of replaning causes used to decide wether to replan
     */
    std::vector<replaningCause*> replaning_causes;
    int verbosity;		///< Verbosity level of the replanner

};



} // namespace replan

} // namespace cam_exploration


#endif  // CAM_EXPLORATION_REPLAN_H

#ifndef CAM_EXPLORATION_FRONTIER_VALUE_H
#define CAM_EXPLORATION_FRONTIER_VALUE_H


#include <cam_exploration/frontier.h>

/**
 * @file frontierValue.h
 * @brief This file provides a set of objects which aim to evaluate a frontier according to its worthyness for exploration pourposes
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-19
 */


namespace cam_exploration{
/**
 * @brief This namespace hosts all the code related with the exploration strategy
 */
namespace strategy{

/**
 * @brief Base class for frontier evaluation objects
 *
 * It provides a common interface for the different implementations
 */
class frontierValue
{
public:
    /**
     * @brief Get the name of the evaluation object
     *
     * @return Name of the evaluation object
     */
    virtual const char* name() const = 0;
    /**
     * @brief Get the value of \p frontier_in according to certain criteria
     *
     * @param frontier_in Frontier to be evaluated
     *
     * @return Frontier value
     */
    virtual double value(const frontier& frontier_in) const = 0;
    /**
     * @brief Get useful info of \p frontier_in regarding the criteria of the object
     *
     * @param frontier_in Incoming frontier
     *
     * @return Information
     */
    virtual const char* printInfo(const frontier& frontier_in) const;

    float weight;	///< Value by which the object value should be weighted
};


/**
 * @brief Frontier evaluation object that favours the larger frontiers
 */
class maxSize : public frontierValue
{
public:
    /**
     * @brief Constructor. Gets configuration parameters.
     *
     * @param params Configuration paramteres.
     */
    maxSize(std::map<std::string, std::string> params);
    const char* name() const;
    /**
     * @brief Evaluate the length of a frontier
     *
     * @param frontier_in Frontier to be evaluated
     * @return Size of \p frontier_in
     */
    double value(const frontier& frontier_in) const;
    const char* printInfo(const frontier&) const;
};


/**
 * @brief Frontier evaluation object that favours the minimum Euclidian distance between the robot position and the frontier::free_center_cell of a frontier
 */
class minEuclidianDistance : public frontierValue
{
public:
    /**
     * @brief Constructor. Gets configuration parameters.
     *
     * @param params Configuration paramteres.
     */
    minEuclidianDistance(std::map<std::string, std::string> params);
    const char* name() const;
    /**
     * @brief Evaluates a frontier
     *
     * The value of a frontier decreases exponencially with the distance multiplied by the factor 1/dispersion
     *
     * @param frontier_in Frontier to be evaluated
     *
     * @return Frontier's value
     */
    double value(const frontier& frontier_in) const;
    const char* printInfo(const frontier&) const;
private:
    /**
     * @brief Computes the Euclidian distance between the robot position and the frontier::free_center_cell of \p frontier_in
     *
     * @param frontier_in Incoming frontier
     *
     * @return Euclidian distance
     */
    double dist(const frontier& frontier_in) const;
    /**
     * @brief Degree of the locality of the evaluation criterion.
     *
     * Large values yield global distance evaluation while small values decrease faster the frontier's value as the Euclidian distance grows
     */
    double dispersion;
};


/**
 * @brief Frontier evaluator based on A*.
 *
 * This evaluator finds the path from the robot postion to a frontier's frontier::free_center_cell and then uses the length of such path to evaluate the frontier. The shorter the path, the better the frontier.
 */
class minAStarDistance : public frontierValue
{
public:
    /**
     * @brief Constructor. Gets configuration parameters.
     *
     * @param params Configuration paramteres.
     */
    minAStarDistance(std::map<std::string, std::string> params);
    const char* name() const;
    /**
     * @brief Evaluates a frontier
     *
     * The value of a frontier decreases exponencially with the distance multiplied by the factor 1/dispersion
     *
     * @param frontier_in Frontier to be evaluated
     *
     * @return Frontier's value
     */
    double value(const frontier& frontier_in) const;
    const char* printInfo(const frontier&) const;
private:
    /**
     * @brief Finds the A* path and gives its distance
     *
     * @param frontier_in Frontier to be evaluated
     *
     * @return Path length
     */
    double dist(const frontier& frontier_in) const;
    /**
     * @brief Degree of the locality of the evaluation criterion.
     *
     * Large values yield global distance evaluation while small values decrease faster the frontier's value as the A* path lenth grows
     */
    double dispersion;
};


} /* strategy */
} /*cam_exploration*/


#endif  // CAM_EXPLORATION_FRONTIER_VALUE_H


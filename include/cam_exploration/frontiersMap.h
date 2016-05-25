#ifndef CAM_EXPLORATION_FRONTIERS_MAP_H
#define CAM_EXPLORATION_FRONTIERS_MAP_H


#include <cam_exploration/frontierValue.h>
#include <list>

/**
 * @file frontiersMap.h
 * @brief This file basically hosts the class cam_exploration::FrontiersMap
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-19
 */


namespace cam_exploration{


/**
 * @brief Collectoion of frontier evaluation objects
 */
typedef std::vector<strategy::frontierValue*> fvalueType;

/**
 * @brief Frontier comparaison functor
 */
struct compFrontierValues
{
    /**
     * @brief Default Constructor
     *
     * @param fvalues Frontier evaluation vector. Each element corresponds to a measure of the value of a frontier
     */
    compFrontierValues(fvalueType fvalues) : fvaluesStruct_(fvalues) {}
    /**
     * @brief Compare two functions according to fvalues
     *
     * @param f1 Left Hand Side frontier
     * @param f2 Right Hand Side frontier
     *
     * @return True if \p f1 is less valuable than \p f2, false otherwise
     */
    bool operator()(const frontier& f1, const frontier& f2);
private:
    /**
     * @brief fvalueType to be used for comparison
     */
    fvalueType fvaluesStruct_;
};


/**
 * @brief Functor indicating wether a certain frontier has a minimum size
 */
struct hasNotMinumumSize
{
    /**
     * @brief Constructor
     *
     * @param siz Minimum frontier size
     */
    hasNotMinumumSize(unsigned int siz) : siz_(siz){}
    /**
     * @brief Indicate wether the frontier \p f has not a minimum size
     *
     * @param f Frontier to be evaluated
     *
     * @return True if the frontier size is less than the threshold. False otherwise
     */
    bool operator()(const frontier& f){ return f.size() < siz_; }
private:
    /**
     * @brief Size threshold
     */
    unsigned int siz_;

};


/**
 * @brief Representation of the collection of frontiers in a map
 */
class FrontiersMap
{
public:
    FrontiersMap();		///< Parameterless constructor
    ~FrontiersMap();		///< Parameterless destructor

    /**
     * @brief Append frontier \p f to FrontiersMap::frontiers_
     *
     * @param f New frontier
     */
    void add(frontier f) { frontiers_.push_back(f);}

    /**
     * @brief Copy the content of \p frontiers_in to FrontiersMap::frontiers_
     *
     * @param frontiers_in Container of frontiers
     */
    template <typename T>
    void setFrontiers(T frontiers_in)
    {
    	frontiers_.clear();
    	// Curously enough, copy_if does not exist previous to C++11 because of a Stroustroup's oversight
    	// One have to use this 'copy_if_not'
	std::remove_copy_if(frontiers_in.begin(), frontiers_in.end(), std::back_inserter(frontiers_),
			hasNotMinumumSize(minimum_size_));
    }

    /**
     * @brief Print information of all frontiers regarding their evaluations
     */
    void printAll();
    /**
     * @brief Get the most valued frontier
     *
     * @return Most valued frontier
     */
    frontier max();

    /**
     * @brief Get parameters from parameter server
     *
     * @param nptr Pointer from which to retreive parameters. It should have the proper namespace
     */
    void getParams(ros::NodeHandlePtr nptr);
    /**
     * @brief Add a frontier evaluation object
     *
     * @param name Name of the frontier evaluation object
     * @param params Set of parameters to be passed to the evaluation object
     */
    void addFrontierValue(std::string name, std::map<std::string, std::string> params);
    /**
     * @brief Add a frontier evaluation object without parameters
     *
     * @param name Name of the frontier evaluation object
     */
    void addFrontierValue(std::string name);

    /**
     * @brief Append frontier evaluation object to FrontiersMap::fvalues
     *
     * @param fvalue New frontier evaluation object to be considered
     */
    void addStrategy(strategy::frontierValue & fvalue)
    {
    	fvalues.push_back(&fvalue);
    }

    /**
     * @brief Check if the OccupancyGrid cell \p c is frontier cell
     *
     * @param c OccupancyGrid cell to be evaluated
     *
     * @return True if \p c corresponds to a frontier, false otherwise
     */
    bool isFrontier(int c);

    /**
     * @brief Verbosity level
     */
    int verbosity;

private:
    std::list<frontier> frontiers_;			///< Container of frontiers

    std::vector<strategy::frontierValue*> fvalues;	///< Collection of frontier evaluation objects

    bool isConfigured;					///< Is the map of frontiers configured? (e.g. has getParams been called?)
    unsigned int minimum_size_;				///< Which is the minimum size a frontier should has in order to be considered?
};


} /* cam_exploration */




#endif  // CAM_EXPLORATION_FRONTIERS_MAP_H


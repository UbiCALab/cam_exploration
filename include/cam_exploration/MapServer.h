#ifndef CAM_EXPLORATION_MAP_SERVER_H
#define CAM_EXPLORATION_MAP_SERVER_H


/**
 * @file MapServer.h
 * @brief This file mainly hosts the class cam_exploration::MapServer
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-21
 */


#include "cam_exploration/frontiersMap.h"

namespace cam_exploration{


/**
 * @brief RGBD camera's projection map handler. It is intended to be used with the output "projj_map" of rtabmap
 *
 * 	This class also provides facilities for frontier finding and mantains the set of frontiers present in the map
 */
class MapServer
{
public:
    /**
     * @brief Parameterless constructor
     */
    MapServer(){}
    /**
     * @brief Subscribe the server to a certain topic and get the parameters from the Parameter Server
     *
     * @param topic Topic to which the Map Server should be subscribed
     * @param frontiersMapCallback Callback function called whien an update of the MapServer::fmap happens
     * @param nptr Node handler pointer used for subscription
     * @param nptr_private Node handler used for getting parameters from the parameter server. Note that it may have different namespace than \p nptr
     */
    void subscribeMap(const char* topic, void (*frontiersMapCallback)(FrontiersMap&), ros::NodeHandlePtr nptr, ros::NodeHandlePtr nptr_private);
    /**
     * @brief Checks wether a new OccupancyGrid has been received
     *
     * @return True if that is the case, false otherwise
     */
    bool mapReceived() { return map_received; }
    /**
     * @brief Flag a new incoming map
     */
    void setMapReceived() { map_received = true; }

    /**
     * @brief Check wether a point is a valid goal for the robot
     *	The corresponding cell has to be in the map and be free, as well as its neghbour cells
     *
     * @param Point The candidate goal
     *
     * @return True if it is a proper goal, false otherwise
     */
    bool isValidPoint(const geometry_msgs::Point &Point);
    /**
     * @brief Check wether a cell is a valid goal for the robot
     *	The corresponding cell has to be in the map and be free, as well as its neghbour cells
     *
     * @param c Index to the candidate goal cell
     *
     * @return True if it is a proper goal, false otherwise
     */
    bool isValidCell(int c) { return isValidPoint(cell2point(c)); }
    /**
     * @brief Check wether a cell is free in the OccupancyGrid
     *
     * @param c Index of the cell to be considered
     *
     * @return True if the cell is free, false otherwise
     */
    bool isFree(int c);
    /**
     * @brief Check wether a point is free in the OccupancyGrid
     *
     * @param Point Point to be considered. In the same frame as MapServer::map
     *
     * @return True if the corresponding cell is free
     */
    bool isFree(const geometry_msgs::Point &Point);

    /**
     * @brief Gets the height of the OccupancyGrid
     *
     * @return The height of the map
     */
    inline int height() { return map.info.height; }
    /**
     * @brief Gets the withd of the OccupancyGrid
     *
     * @return The width of the map
     */
    inline int width() { return map.info.width; }

    // Auxiliar functions
    /**
     * @brief Gets the surrounding cells of a certain cell in a 4-connecivity basis
     *
     * @param point Center cell
     * @param points[] Surrounding cells. To be filled by the function
     *
     * @see getAdjacentPoints(int, int[])
     */
    void getStraightPoints(int point, int points[]);
    /**
     * @brief Gets the surrounding cells of a certain cell in a 8-connectivity basis
     *
     * @param point Center cell
     * @param points[] Surrounding cells. To be filled by the function
     *
     * @see getStraightPoints(int, int[])
     */
    void getAdjacentPoints(int point, int points[]);
    /**
     * @overload
     *
     * @returns Surrounding cells
     */
    std::vector<int> getAdjacentPoints(int point);
    /**
     * @brief Gets a grid of map cells centered in a certain cell
     * 	Such grid is computed by adding surrounding concentric squares.
     *<pre>
     * 		Example 1:
     *			depth: 1
     *			center: c
     *			computed grid:
     </pre>
     *
     *				* * *
     *				* c *
     *				* * *
     *
     *<pre>
     * 		Example 2:
     *			depth: 3
     *			center: c
     *			computed grid:
     </pre>
     *
     *			      * * * * * * *
     *			      * * * * * * *
     *			      * * * * * * *
     *			      * * * c * * *
     *			      * * * * * * *
     *			      * * * * * * *
     *			      * * * * * * *
     * <pre></pre>
     *	The cells of the grid are marked with '*'. The center cell is included.
     *
     * @param center Grid's center
     * @param depth Number of consecutive concentric squares formed by cells to be added in the final grid
     *
     * @return Cells of the corresponding centered square of cells
     */
    std::vector<int> getNeighbours(int center, unsigned int depth);
    /**
     * @copydoc getNeighbours(int, unsigned int)
     */
    std::vector<int> getNeighbours(geometry_msgs::Point center, unsigned int depth) { return getNeighbours(point2cell(center), depth); }
    /**
     * @brief Checks wether a certain map cell is part of a frontier
     *
     * @param cell Cell to be considered
     *
     * @return True if that's the case
     */
    bool isFrontierCell(int cell);

    /**
     * @brief Conversion function
     *
     * @param point A point in the XY plane
     *
     * @return Index of the cell in the map corresponding to \p point
     *
     * @see cell2point(const int&)
     */
    int point2cell(const geometry_msgs::Point & point);
    /**
     * @brief Conversion function
     *
     * @param cell Index of a cell in the map
     *
     * @return Point indicating the position of \p cell in the space.
     *
     * @see point2cell(int)
     */
    geometry_msgs::Point cell2point(const int & cell);

private:
    // Attributes
    /**
     * @brief Subscriber to the map
     */
    static ros::Subscriber sub;
    /**
     * @brief Map to be handled. This is the main variable of the class
     */
    static nav_msgs::OccupancyGrid map;
    /**
     * @brief It is a map containing all frontiers points
     */
    static nav_msgs::OccupancyGrid frontier_map;
    /**
     * @brief The object to handle the collection of frontiers
     */
    static FrontiersMap fmap;

    /**
     * @brief Tells wether a new map has been received and not yet handled
     */
    static bool map_received;
    /**
     * @brief Tells wether the server has already subscribed to the map
     */
    static bool is_subscribed;
    /**
     * @brief Function callback tu update the frontiers map
     */
    void (*functionCallback)(FrontiersMap&);

    // Frontier handling functions
    /**
     * @brief Handles a new incoming map
     *
     * @param msg Pointer to the new map
     */
    void occupancy_gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    /**
     * @brief Find the frontiers in the map
     */
    void findFrontiers();
    /**
     * @brief Check wether a cell corresponds to any frontier
     *
     * @param cell Index of the cell to be handled
     *
     * @return True if the cell corresponds to a frontier
     */
    bool isFrontier(int cell);
    /**
     * @brief Labels a certain image according to the Two Pass Labeling algorithm.
     *	Information about this algorithm can be found in <a href="https://en.wikipedia.org/wiki/Connected-component_labeling">Wikipedia</a>
     *
     * @param image to be labeled
     */
    void twoPassLabeling(std::vector<int>& image);

    int left(int point);		///< @brief Gets left cell	@param point cell to be handled 	@return The cell at the left
    int upleft(int point);		///< @brief Gets upleft cell	@param point cell to be handled 	@return The cell at the upleft
    int up(int point);			///< @brief Gets up cell	@param point cell to be handled 	@return The cell at the up
    int upright(int point);		///< @brief Gets upright cell	@param point cell to be handled 	@return The cell at the upright
    int right(int point);		///< @brief Gets right cell	@param point cell to be handled 	@return The cell at the right
    int downright(int point);		///< @brief Gets downright cell	@param point cell to be handled 	@return The cell at the downright
    int down(int point);		///< @brief Gets down cell	@param point cell to be handled 	@return The cell at the down
    int downleft(int point);		///< @brief Gets downleft cell	@param point cell to be handled 	@return The cell at the downleft
    /**
     * @brief Rounds a float to the nearest integer closer to 0
     *
     * @param value Incoming float
     *
     * @return Rounded value
     */
    int floor0(float value);

};



} //End namespace cam_exploration

#endif



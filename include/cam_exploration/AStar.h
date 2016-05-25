
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

#ifndef CAM_EXPLORATION_ASTAR_H_
#define CAM_EXPLORATION_ASTAR_H_

/**
 * @file AStar.h
 * @brief AStar library file. This file basically hosts the classes cam_exploration::strategy::AStar and cam_exploration::strategy::cell
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-19
 */



#include <cam_exploration/MapServer.h>

namespace cam_exploration{
namespace strategy{


/**
 * @brief Unsigned integer type used for A* distances and costs
 */
typedef unsigned int aInt;


/**
 * @brief Cell object stored in the A* algorithm
 */
struct cell
{
    int id;						///< Cell identifier
    int parent;						///< Parent cell identifier

    aInt g_cost;					///< Cost to go to the cell from the start cell
    aInt h_cost;					///< Heuristic cost. Rectangular distance to the goal cell
    bool closed;					///< Is the cell int the closed list?
    aInt cost() const { return g_cost + h_cost; }	///< Total cost of cell (F cost)

};


/**
 * @brief Implementation of the A* search algorithm
 *
 * An explanation can be found in <a href="https://en.wikipedia.org/wiki/A*_search_algorithm">Wikipedia</a>
 */
class AStar
{
public:

    /**
     * @brief Class constructor. Needs end cell.
     *
     * @param g End cell
     */
    AStar(int g) { init(g); }
    /**
     * @brief Class constructor. Needs end cell.
     *
     * @param p End cell
     */
    AStar(geometry_msgs::Point p) { init(map_server.point2cell(p)); }

    /**
     * @brief Set the end cell
     *
     * @param g End cell
     */
    void setGoal(int g) { goal.id = g; }
    /**
     * @brief Perform the algorithm and find a path between start and goal
     *
     * @param start Start cell
     *
     * @return Distance of the found path
     */
    aInt distance(int start);

private:
    cell goal;						///< Goal cell
    void init(int);					///< Initialise class
    std::vector<cell> cells;				///< Vector of consifered open and closed cells

    MapServer map_server;				///< MapServer handler

    int cost_rec;					///< Cost to go vertically or horizontally
    int cost_ver;					///< Cost to go in diagonal
    int cost_rec_h;					///< Heuristic cost in vertical or horizontal
    int cost_ver_h;					///< Heuristic cost in diagonal

    inline int height() { return map_server.height(); }	///< Map's height
    inline int width() { return map_server.width(); }	///< Map's width

    /**
     * @brief Computes the number of open cells
     *
     * @return Number of open cells
     */
    aInt nOpen();

    /**
     * @brief Gets the lowest valued cell
     *
     * @return Pointer to the lowest valued cell
     */
    cell* getLowest();
    /**
     * @brief Handle the neighbours of cell \p c
     *
     * @param c Center cell to be considered
     *
     * @return False if none of the neighbours is valid, true otherwise.
     */
    bool loopNeighbours(cell& c);
    /**
     * @brief Perform the algorithm between \p start and goal
     *
     * @param start Starting cell
     *
     * @return Pointer to the goal cell in cells
     */
    cell * solve(int start);
    /**
     * @brief Get the heuristic cost of cell with id \p cell_id
     *
     * @param cell_id Reference Cell
     *
     * @return Heuristic cost
     */
    aInt getH(int cell_id);
    /**
     * @brief Get the cost of the cell \p c
     *
     * @param c Cell which cost is to be computed
     *
     * @return Total cost of c
     */
    aInt cellCost(const cell& c) { return c.cost(); }

};


} /* cam_exploration */
} /* strategy */

#endif  // CAM_EXPLORATION_ASTAR_H_

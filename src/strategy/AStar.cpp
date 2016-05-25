
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

#include <cam_exploration/AStar.h>

/**
 * @file AStar.cpp
 * @brief AStar library implementation
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-19
 */



using namespace std;

namespace cam_exploration{
namespace strategy {


/**
 * @brief Functor that checks wether a cell has a certain identifier
 */
struct hasId
{
    /**
     * @brief Constructor
     *
     * @param id Identifier to be checked
     */
    hasId(int id) : id_(id) {}
    /**
     * @brief Checking implementation
     *
     * @param c cell to be evaluated
     *
     * @return True if has the desired identifier, false otherwise
     */
    bool operator()(cell c) { return c.id == id_;}
private:
    /**
     * @brief Desired identifier
     */
    int id_;
};

void AStar::init(int i)
{
    goal.id = i;

    cost_rec = cost_rec_h = 10;
    cost_ver = cost_ver_h = 14;
    MarkerPublisher markers;
    markers.add("AStar", "AStar");
}


aInt AStar::distance(int start)
{
    cell * c = solve(start);
    cell * current = c;

    if (current == NULL){
            //ROS_INFO("Not a valid point. N. cells: %lu. Open cells: %u", cells.size(), nOpen());
    	return 0;
    }

    //ROS_INFO("Valid point! [id, parent] : [%d, %d]", current->id, current->parent);
    // Measuring the path back to the start
    aInt dist = 0;
    bool found = false;
    vector<geometry_msgs::Point> path;
    while (!found){
	dist++;
	path.push_back(map_server.cell2point(current->id));

    	vector<cell>::iterator it = find_if(cells.begin(), cells.end(), hasId(current->parent));

	if (it == cells.end())
	    found = true;
	else
	    current = &*it;
    }
    //ROS_INFO("Valid distance: %d", dist);

    MarkerPublisher markers;
    markers.publish("AStar", path);

    return dist;
}


cell * AStar::solve(int start)
{
    //ROS_INFO("Solving AStar from %d to %d...", start, goal.id);
    cell start_cell;
    start_cell.id = start;
    start_cell.closed = false;
    start_cell.g_cost = 0;
    start_cell.h_cost = getH(start);
    start_cell.parent = -1;

    cells.push_back(start_cell);

    // Main loop
    bool found = false;
    cell * current;
    unsigned long int i = 0;
    while (!found){
    	current = getLowest();
	current->closed = true;


	if (current->id == goal.id){
	    found = true;
	}
	else{
	    if (!loopNeighbours(*current)){
		return NULL;
	    }
	}
	++i;
    }

    return current;
}



aInt AStar::getH(int i)
{
    int diff = abs(i - goal.id);
    int x = diff % width();
    int y = diff / width();

    return min(x, y) * cost_ver_h + abs(x - y) * cost_rec_h;
}

bool AStar::loopNeighbours(cell& c)
{
    vector<int> neighs = map_server.getAdjacentPoints(c.id);

    bool adding = false;
    // Loop all neighbours
    for (vector<int>::iterator i = neighs.begin(); i != neighs.end(); ++i){

	if (!map_server.isValidCell(*i)){
	    continue;
	}

	// Compute new g cost
	int dist;
	aInt new_cost;
	dist = abs(*i - c.id);
	if (abs(dist - width()) == 1)
	    new_cost = c.g_cost + cost_ver;
	else
	    new_cost = c.g_cost + cost_rec;


	// Check if beighbour is new
	vector<cell>::iterator ic = find_if(cells.begin(), cells.end(), hasId(*i));
	bool is_new = ic == cells.end();


	if (!is_new && (ic->closed || ic->cost() < new_cost)){
	    continue;
	}
	adding = true;

	if (is_new){
	    // Create new cell
	    cell new_cell;
	    new_cell.id = *i;
	    new_cell.parent = c.id;
	    new_cell.closed = false;
	    new_cell.h_cost = getH(*i);
	    new_cell.g_cost = new_cost;
	    cells.push_back(new_cell);
	}
	else{
	    ic->g_cost = new_cost;
	    ic->parent = c.id;
	}
    }

    return adding || nOpen() != 0;
}


/**
 * @brief Get the cost of a cell
 *
 * In the case that the cell is not free, return the maximum cost allowed
 *
 * @param c Cell to be evaluated
 *
 * @return Cell's cost
 */
aInt freeCellCost(const cell& c)
{
    return c.closed ? numeric_limits<aInt>::max() : c.cost();
}
/**
 * @brief Check if \p c1 has less cost than \p c2
 *
 * @param c1 Cell 1
 * @param c2 Cell 2
 *
 * @return True if the first has lower cost, false otherwise
 */
bool lessThan(const cell& c1, const cell& c2)
{
    return freeCellCost(c1) < freeCellCost(c2);
}
cell* AStar::getLowest()
{
    return &*min_element(cells.begin(), cells.end(), lessThan);
}


/**
 * @brief Check wether a cell is open
 *
 * @param c Cell to be evaluated
 *
 * @return True if the cell is open, false otherwise
 */
bool isOpen(const cell& c) { return !c.closed; }
aInt AStar::nOpen()
{
    return count_if(cells.begin(), cells.end(), isOpen);
}


} /* cam_exploration */

} // namespace strategy

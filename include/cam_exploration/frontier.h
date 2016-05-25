#ifndef CAM_EXPLORATION_FRONTIER_H
#define CAM_EXPLORATION_FRONTIER_H


#include <vector>
#include <algorithm>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "cam_exploration/MarkerPublisher.h"

#include <algorithm>
#include <limits>
#include <cmath>
#include <math.h>

/** 
 * @file frontier.h
 * @brief This file hosts the cam_exploration::frontier class
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-19
 */



namespace cam_exploration{

    /**
     * @brief Representation of a 2D frontier for exploration pourposes
     */
    class frontier
    {
        public:
            int id;							///< Frontier identifier
            geometry_msgs::Point center_point;                  	///< Position of the center cell
            int center_cell;						///< Frontier's center cell
            geometry_msgs::Point free_center_point;             	///< Position of the free_center_cell
            int free_center_cell;					///< Nearest free cell to the center_cell
            std::vector<int> cells;                                  	///< Frontiers cells in the OccupancyGrid
            std::vector<geometry_msgs::Point> points;                	///< Position of the cells

            /**
             * @brief Get number of cells
             *
             * @return Number of cells
             */
            unsigned int size() const{return cells.size();}
            /**
             * @brief Publish cells according to \p publishCells
             *
             * @param publishCells Publishing function
             */
            void publish(void (*publishCells)(const std::vector<int> &));
            /**
             * @brief Publish the center_cell according to \p publishCells
             *
             * @param publishCells Publishing function
             */
            void publishCenter(void (*publishCells)(const std::vector<int> &));

    };

}

#endif

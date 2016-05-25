#include "cam_exploration/frontier.h"


/** 
 * @file frontier.cpp
 * @brief Implementation of frontier.h
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-21
 */


namespace cam_exploration{


void frontier::publish(void (*publishCells)(const std::vector<int> &))
{
    publishCells(this->cells);
}

void frontier::publishCenter(void (*publishCells)(const std::vector<int> &))
{
    std::vector<int> v(this->free_center_cell);
    publishCells(v);
}



} // End of cam_exploration namespace

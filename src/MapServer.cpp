#include "cam_exploration/MapServer.h"

#include <boost/pending/disjoint_sets.hpp>
#include <deque>


/** 
 * @file MapServer.cpp
 * @brief Implementation of the MapServer.h file
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-21
 */


using namespace std;
//using namespace cam_exploration::strategy;
namespace cam_exploration{

// Static members
bool MapServer::map_received = false;
bool MapServer::is_subscribed = false;
nav_msgs::OccupancyGrid MapServer::map;
nav_msgs::OccupancyGrid MapServer::frontier_map;
FrontiersMap MapServer::fmap;
ros::Subscriber MapServer::sub;

void MapServer::subscribeMap(const char *topic, void (*fcCallback)(FrontiersMap&), ros::NodeHandlePtr n,
					ros::NodeHandlePtr n_private)
{
    if (is_subscribed){
    	ROS_WARN("The Map Server will not subscribe to topic '%s' since it is already subscribed to a topic", topic);
    	return;
    }
    sub = n->subscribe(topic, 1, &MapServer::occupancy_gridCallback, this);
    functionCallback = fcCallback;

    fmap.getParams(n_private);

    is_subscribed = false;
}


//Tells if given point could be a valid goal for the robot: inside map limits, without obstacles around
bool MapServer::isValidPoint(const geometry_msgs::Point & point)
{
  int N = ceil(0.2/map.info.resolution); // radius (in cells) of the robot footprint
  int index = point2cell(point); // cell corresponding to the point

  if(index==-1 || map.data[index]!=0)
    return false; // not valid if the point is out of the map or if the cell is not free

  // Check if no obstacles cells are in the robot footprint
  for(int j=-N; j<=N; j++)
  {
    for(int i=-N; i<=N; i++)
    {
      int cell_i = i*map.info.width+j+index;
      if(cell_i >0 && cell_i < signed(map.data.size())) // check if cell_i is in the map.data size
        if(map.data[cell_i]==100)
          return false; //if the cell_i cell is an obstacle the goal is not valid
    }
  }
  // otherwise the goal is valid
  return true;
}

bool MapServer::isFree(int index)
{
    return map.data[index] <= 10;
    //return isValidPoint(cell2point(index));
}


bool MapServer::isFrontierCell(int cell)
{
    return fmap.isFrontier(cell);
}


bool MapServer::isFree(const geometry_msgs::Point & point)
{
    int index = point2cell(point);
    return isFree(index);
}


// Occupancy Grid Callback: called each time a map message is received
void MapServer::occupancy_gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map=*msg;
    int num_mapcells = map.info.width*map.info.height;
    frontier_map = *msg;
    map_received=true;

    frontier_map.data.resize(num_mapcells);
    findFrontiers();
}


// FRONTIER MANAGING FUNCITONS /////////////////////////////////////////////////
void MapServer::findFrontiers()
{
  frontier_map.data.assign(frontier_map.data.size(), 0);
  for(unsigned int i = 0; i < frontier_map.data.size(); ++i)
  {
    if(isFrontier(i))
      frontier_map.data[i] = 100;
    else
      frontier_map.data[i] = 0;
  }

  // publish frontier map
  ros::NodeHandle n;
  ros::Publisher  pub2 = n.advertise<nav_msgs::OccupancyGrid>("frontiersmap", 1);
  pub2.publish(frontier_map);

  // Label frontiers (connected cells)
  vector<frontier> frontiers;
  vector<int> labels;
  twoPassLabeling(labels);

  // Create frontiers structs
  for (unsigned int i = 0; i < labels.size(); ++i)
  {
    if(labels[i]!=0) //frontier labeled cell
    {
      // search for existing frontier
      bool new_label = true;
      for (unsigned int j = 0; j < frontiers.size(); j++)
      {
        // found
        if (frontiers[j].id == labels[i])
        {
          frontiers[j].cells.push_back(i);
          frontiers[j].points.push_back(cell2point(i));
          new_label = false;
          break;
        }
      }
      // not found
      if (new_label)
      {
        frontiers.resize(frontiers.size()+1);
        frontiers.back().cells.push_back(i);
        frontiers.back().points.push_back(cell2point(i));
        frontiers.back().id = labels[i];
      }
    }
  }
  // Search middle cell
  for(unsigned int i = 0; i < frontiers.size(); ++i)
  {
    int label = frontiers[i].id;

    // order the frontier cells
    std::deque<int> ordered_cells(0);
    ordered_cells.push_back(frontiers[i].cells.front());
    while (ordered_cells.size() < frontiers[i].size())
    {
        size_t initial_size = ordered_cells.size();

        // connect cells to first cell
        int frontAdjacentPoints[8];
        getAdjacentPoints(ordered_cells.front(), frontAdjacentPoints);
        for (unsigned int k = 0; k<8; k++)
        {
            if (frontAdjacentPoints[k] != -1 && labels[frontAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), frontAdjacentPoints[k]) == ordered_cells.end() )
            {
                ordered_cells.push_front(frontAdjacentPoints[k]);
                break;
            }
        }

        // connect cells to last cell
        int backAdjacentPoints[8];
        getAdjacentPoints(ordered_cells.back(), backAdjacentPoints);
        for (unsigned int k = 0; k<8; k++)
        {
            if (backAdjacentPoints[k] != -1 && labels[backAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), backAdjacentPoints[k]) == ordered_cells.end() )
            {
                ordered_cells.push_back(backAdjacentPoints[k]);
                break;
            }
        }

        if (initial_size == ordered_cells.size() && ordered_cells.size() < frontiers[i].size())
            break;
    }
    // center cell
    frontiers[i].center_cell = ordered_cells[ordered_cells.size() / 2];

    // find the close free cell of the middle frontier cell
    int adjacentPoints[8];
    getAdjacentPoints(frontiers[i].center_cell, adjacentPoints);
    for (unsigned int k = 0; k<8; k++)
    {
        if (map.data[adjacentPoints[k]] == 0) // free neighbor cell
        {
          frontiers[i].free_center_cell = adjacentPoints[k];
          break;
        }
        if (k == 7)
          ROS_ERROR("findFrontiers: No free cell close to the center frontier cell!");
    }
    frontiers[i].center_point = cell2point(frontiers[i].center_cell);
    frontiers[i].free_center_point = cell2point(frontiers[i].free_center_cell);
    //publishMarker(frontiers[i].id, frontiers[i].free_center_point.x, frontiers[i].free_center_point.y, frontiers[i].id, 0, 0);
  }


  fmap.setFrontiers(frontiers);
  functionCallback(fmap);
}

// Check if a cell is frontier
bool MapServer::isFrontier(int cell)
{
  if(map.data[cell] == -1) //check if it is unknown
  {
    int straightPoints[4];
    getStraightPoints(cell,straightPoints);
    for(int i = 0; i < 4; ++i)
      if(straightPoints[i] != -1 && map.data[straightPoints[i]] == 0) //check if any neigbor is free space
        return true;
  }
  // If it is obstacle or free can not be frontier
  return false;
}


// Two pass labeling to label frontiers [http://en.wikipedia.org/wiki/Connected-component_labeling]
void MapServer::twoPassLabeling(vector<int>& labels)
{
  labels.assign(frontier_map.data.begin(), frontier_map.data.end());
  vector<int> neigh_labels;
  vector<int> rank(1000);
  vector<int> parent(1000);
  boost::disjoint_sets<int*,int*> dj_set(&rank[0], &parent[0]);
  int current_label_=1;

  // 1ST PASS: Assign temporary labels to frontiers and establish relationships
  for(unsigned int i = 0; i < frontier_map.data.size(); i++)
  {
    if( frontier_map.data[i] != 0)
    {
      neigh_labels.clear();
      // Find 8-connectivity neighbours already labeled
      if(upleft(i)  != -1 && labels[upleft(i)]  != 0) neigh_labels.push_back(labels[upleft(i)]);
      if(up(i)      != -1 && labels[up(i)]      != 0) neigh_labels.push_back(labels[up(i)]);
      if(upright(i) != -1 && labels[upright(i)] != 0) neigh_labels.push_back(labels[upright(i)]);
      if(left(i)    != -1 && labels[left(i)]    != 0) neigh_labels.push_back(labels[left(i)]);

      if(neigh_labels.empty())                                                  // case: No neighbours
      {
        dj_set.make_set(current_label_);                                        //   create new set of labels
        labels[i] = current_label_;                                            //   update cell's label
        current_label_++;                                                       //   update label
      }
      else                                                                      // case: With neighbours
      {
        labels[i] = *std::min_element(neigh_labels.begin(), neigh_labels.end());//   choose minimum label of the neighbours
        for(unsigned int j = 0; j < neigh_labels.size(); ++j)                   //   update neighbours sets
          dj_set.union_set(labels[i],neigh_labels[j]);                          //   unite sets minimum label with the others
      }
    }
  }

  // 2ND PASS: Assign final label
  dj_set.compress_sets(labels.begin(), labels.end());
  // compress sets for efficiency
  for(unsigned int i = 0; i < frontier_map.data.size(); i++)
    if( labels[i] != 0)
      labels[i] = dj_set.find_set(labels[i]);                                 // relabel each element with the lowest equivalent label

}


int MapServer::point2cell(const geometry_msgs::Point & point)
{
  if(point.x <= map.info.origin.position.x || point.x >= map.info.width*map.info.resolution + map.info.origin.position.x  ||
     point.y <= map.info.origin.position.y || point.y >= map.info.height*map.info.resolution+ map.info.origin.position.y)
  {
    return -1;
  }

  int x_cell = floor0((point.x - map.info.origin.position.x)/map.info.resolution);
  int y_cell = floor0((point.y - map.info.origin.position.y)/map.info.resolution);
  int cell = x_cell + (y_cell)*map.info.width;
  return cell;
}
geometry_msgs::Point MapServer::cell2point(const int & cell)
{
  geometry_msgs::Point point;
  point.x = (cell % map.info.width)*map.info.resolution + map.info.origin.position.x + map.info.resolution / 2;
  point.y = floor(cell/map.info.width)*map.info.resolution + map.info.origin.position.y + map.info.resolution / 2;
  return point;
}

void MapServer::getStraightPoints(int cell, int cells[])
{
  cells[0] = left(cell);
  cells[1] = up(cell);
  cells[2] = right(cell);
  cells[3] = down(cell);
}
void MapServer::getAdjacentPoints(int cell, int cells[])
{
  cells[0] = left(cell);
  cells[1] = up(cell);
  cells[2] = right(cell);
  cells[3] = down(cell);
  cells[4] = upleft(cell);
  cells[5] = upright(cell);
  cells[6] = downright(cell);
  cells[7] = downleft(cell);
}
vector<int> MapServer::getAdjacentPoints(int cell)
{
  vector<int> cells(8);
  cells[0] = left(cell);
  cells[1] = up(cell);
  cells[2] = right(cell);
  cells[3] = down(cell);
  cells[4] = upleft(cell);
  cells[5] = upright(cell);
  cells[6] = downright(cell);
  cells[7] = downleft(cell);
  return cells;
}
vector<int> MapServer::getNeighbours(int cell, unsigned int depth)
{
    // Get a neighbour of cells centered in cell
    vector<int> neighs;//(num_cells);
    neighs.push_back(cell);
    int corners [4];
    int last_pushed [4];
    fill_n(corners, 4, cell);

    for(unsigned int i=0; i<depth; ++i){
        corners[0] = up(corners[0]);
        corners[1] = right(corners[1]);
        corners[2] = down(corners[2]);
        corners[3] = left(corners[3]);
        copy(corners, corners+4, last_pushed);

        for(unsigned int j = 0; j<4; ++j){
            neighs.push_back(corners[j]);
        }

        for(unsigned int j = 0; j<i; ++j){
            last_pushed[0] = downright(last_pushed[0]);
            last_pushed[1] = downleft(last_pushed[1]);
            last_pushed[2] = upleft(last_pushed[2]);
            last_pushed[3] = upright(last_pushed[3]);

            for(int k = 0; k<4; ++k){
                neighs.push_back(last_pushed[k]);
            }
        }
    }
    return neighs;
}


int MapServer::right(int cell)
{
  // only go left if no index error and if current cell is not already on the left boundary
  if((cell % map.info.width != 0))
    return cell+1;

  return -1;
}
int MapServer::upright(int cell)
{
  if((cell % map.info.width != 0) && (cell >= (int)map.info.width))
    return cell-map.info.width+1;

  return -1;
}
int MapServer::up(int cell)
{
  if(cell >= (int)map.info.width)
    return cell-map.info.width;

  return -1;
}
int MapServer::upleft(int cell)
{
  if((cell >= (int)map.info.width) && ((cell + 1) % (int)map.info.width != 0))
    return cell-map.info.width-1;

  return -1;
}
int MapServer::left(int cell)
{
  if((cell + 1) % map.info.width != 0)
    return cell-1;

  return -1;
}
int MapServer::downleft(int cell)
{
  if(((cell + 1) % map.info.width != 0) && ((cell/map.info.width) < (map.info.height-1)))
    return cell+map.info.width-1;

  return -1;
}
int MapServer::down(int cell)
{
  if((cell/map.info.width) < (map.info.height-1))
    return cell+map.info.width;

  return -1;
}
int MapServer::downright(int cell)
{
  if(((cell/map.info.width) < (map.info.height-1)) && (cell % map.info.width != 0))
    return cell+map.info.width+1;

  return -1;
}
int MapServer::floor0(float value)
{
  if (value < 0.0)
    return ceil( value );
  else
    return floor( value );
}




} // End Namespace cam_exploration


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

#include <cam_exploration/frontiersMap.h>


/**
 * @file frontiersMap.cpp
 * @brief Implementation of the frontiersMap.h file
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-21
 */


using namespace cam_exploration::strategy;
using namespace std;

namespace cam_exploration{

FrontiersMap::FrontiersMap()
{
    isConfigured = false;
}
FrontiersMap::~FrontiersMap()
{
    fvalues.clear();
}


void FrontiersMap::getParams(ros::NodeHandlePtr n_parent)
{
    ROS_INFO("Seting parameters of the FrontiersMap");

    ros::NodeHandle n(*n_parent, "frontier_value");

    unsigned int default_minimum_size = 15;

    vector<string> fvalues;
    n.getParam("functions", fvalues);

    if (verbosity > 0){
    	ROS_INFO("Configuring frontier_value");
    	if (fvalues.size() == 0){
	   ROS_ERROR("Parameters of frontier_value are not specified");
	}
    }


    for(vector<string>::iterator it = fvalues.begin(); it != fvalues.end(); ++it){
	map<string, string> parameters;

	if (verbosity > 0)
	    ROS_INFO("    fvalue: %s", it->c_str());

	if (n.getParam(it->c_str(), parameters)){
	    addFrontierValue(it->c_str(), parameters);
	}
	else{
	    ROS_INFO("No parameters found for frontier value %s", it->c_str());
	    addFrontierValue(it->c_str());
	}
    }

    int minimum_size_in;
    if(n.getParam("minimum_frontier_size", minimum_size_in)){
	minimum_size_ = minimum_size_in;
    }else{
    	ROS_WARN("Parameter minimum_frontier_size not set, using default value: %u",
    		default_minimum_size);
    	minimum_size_ = default_minimum_size;
    }

    n.param<int>("frontier_value/verbosity", verbosity, 1);

    isConfigured = true;
}


void FrontiersMap::addFrontierValue(std::string name, std::map<std::string, std::string> parameters)
{
    if (name == "max_size"){
	fvalues.push_back(new maxSize(parameters));
    }
    else if (name == "min_euclidian_distance"){
	fvalues.push_back(new minEuclidianDistance(parameters));
    }
    else if (name == "min_astar_distance"){
	fvalues.push_back(new minAStarDistance(parameters));
    }
    else{
	ROS_ERROR("String %s does not match any valid frontierValue", name.c_str());
    }
}


bool FrontiersMap::isFrontier(int cell)
{
    for (list<frontier>::iterator it = frontiers_.begin(); it != frontiers_.end(); ++it){
    	vector<int>::iterator cit = find(it->cells.begin(), it->cells.end(), cell);
    	if (cit != it->cells.end())
    	    return true;
    }
    return false;
}


void FrontiersMap::addFrontierValue(std::string name)
{
    map<string, string> fakeParams;
    addFrontierValue(name, fakeParams);
}


void FrontiersMap::printAll()
{
    ROS_INFO("-----------------------------------------------------");
    ROS_INFO(" Frontiers information:");
    ROS_INFO(".....................................................");
    for(list<frontier>::iterator it = frontiers_.begin(); it != frontiers_.end(); ++it){
	ROS_INFO("Frontier %u.", it->id);

	for (vector<frontierValue*>::iterator iv = fvalues.begin(); iv != fvalues.end(); ++iv)
	    ROS_INFO("    Name: %*s, Value: %2.f. %s", 15, (**iv).name(), (**iv).value(*it), (**iv).printInfo(*it));
    }
    ROS_INFO("-----------------------------------------------------");
}

frontier FrontiersMap::max()
{
    if(verbosity > 1)
	printAll();

    if (!isConfigured){
    	ROS_ERROR("There are no functions scpecified to choose the best frontier");
    }
    return *max_element(frontiers_.begin(), frontiers_.end(), compFrontierValues(fvalues));
}

bool compFrontierValues::operator()(const frontier& f1, const frontier& f2)
{
    double c1 = 0, c2 = 0;
    for(vector<strategy::frontierValue*>::const_iterator it = fvaluesStruct_.begin(); it != fvaluesStruct_.end(); ++it){
	c1 += (*(*it)).value(f1);
	c2 += (*(*it)).value(f2);
    }
    return c1 < c2;
}


}	/* cam_exploration  */

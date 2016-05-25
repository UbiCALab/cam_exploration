#ifndef CAM_EXPLORATION_MARKER_SERVER_H
#define CAM_EXPLORATION_MARKER_SERVER_H

/**
 * @file MarkerPublisher.h
 * @brief Handles all the marker publications of the project cam_exploration
 * @author Jordi Soler
 * @version 1
 * @date 2016-04-21
 */



#include <ros/ros.h>

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"

namespace cam_exploration {
    
/**
 * @brief Represents a publishable object. It only supports visualisation_msgs::Marker messages
 */
class pub
{
public:
    /**
     * @brief Constructor
     * 	It advertises to publish visualisation_msgs::Marker to the topic \p topic
     * @param name Name representing this object. It is an identifier
     * @param topic Topic to be advertised
     * @param n NodeHandle to advertise the topic \p topic
     * @param m Marker with the properties that the sent messages should have
     */
    pub(const char* name, const char* topic, ros::NodeHandle n, visualization_msgs::Marker m) :
					    topic_(topic), name_(name), n_(n), m_(m) {initialise();}
    /**
     * @brief Constructor
     * 	It advertises to publish visualisation_msgs::Marker to the topic \p topic
     * @param name Name representing this object. It is an identifier
     * @param topic Topic to be advertised
     * @param n NodeHandle to advertise the topic \p topic
     * @param type Type of the visualisation_msgs::Marker to be published
     */
    pub(const char* name, const char* topic, ros::NodeHandle n, int type);

    /**
     * @brief Publish a marker at given point(s)
     *
     * @param p Location of the marker
     */
    void publish(geometry_msgs::Pose p) const { publish_(p);}
    /**
     * @copydoc publish(geometry_msgs::Pose) const
     *
     * @param yaw Yaw angle to deduce orientation in the XY plane
     */
    void publish(geometry_msgs::Point p, int yaw) const;
    /**
     * @copydoc publish(geometry_msgs::Pose) const
     * Uses line strips type
     */
    void publish(std::vector<geometry_msgs::Point> p) const { publish_(p);}

    /**
     * @brief Set some base property(ies) of the marker message to be published
     *
     * @param m Marker from which to copy the properties
     */
    void setProperty(visualization_msgs::Marker m) {m_ = m;}
    /**
     * @copybrief setProperties(visualisation_msgs::Marker)
     *
     * @param type Type of the marker to be published
     */
    void setProperty(int type) {m_.type = type;}
    /**
     * @copybrief setProperties(visualisation_msgs::Marker)
     *
     * @param color Color of the marker to be published
     */
    void setProperty(std_msgs::ColorRGBA color) {m_.color = color;}
    /**
     * @copybrief setProperties(visualisation_msgs::Marker)
     *
     * @param scale Scale of the marker to be published
     */
    void setProperty(geometry_msgs::Vector3 scale) {m_.scale = scale;}

    /**
     * @brief Checks wether the object has a certain identifier
     *
     * @param name Object's identifier
     *
     * @return True if there is a match
     */
    bool is(const char* name) const { return name == name_; }

private:
    const char *topic_;			///< Topic where the messages are published
    const char *name_;			///< Identifier (name) of the object
    ros::NodeHandle n_;			///< NodeHandle to advertise the topic
    visualization_msgs::Marker m_;	///< Base marker message to be publided. Contains the properties to be used
    ros::Publisher p_;			///< Ros publisher to publish the messages

    /**
     * @brief Advertises the topic
     */
    void initialise();
    /**
     * @brief Publishes a unique-point marker
     *
     * @param p Position of the marker
     */
    void publish_(const geometry_msgs::Pose p) const;
    /**
     * @brief Publishes a series-of-points marker
     *
     * @param points Position of the points of the marker
     */
    void publish_(const std::vector<geometry_msgs::Point>& points) const;

};


/**
 * @brief Server class to handle marker publications.
 *	It facilitates the marker publication by storing a collection of publishers with proper defaults
 */
class MarkerPublisher
{
public:
    /**
     * @brief Parameterless constructor. Forwards its work to MarkerPublisher::initialise()
     */
    MarkerPublisher();
    /**
     * @brief Constructor. Initialises the ros::NodeHandle and forwards its work to MarkerPublisher::initialise()
     *
     * @param n_in Node handle to advertise the publishers
     */
    MarkerPublisher(ros::NodeHandle n_in);

    /**
     * @copydoc publish(geometry_msgs::Point,const char*)
     *
     * @param frame Frame to wich the message should be attached
     */
    void publish(geometry_msgs::Point point, const char* topic, const char* frame);
    /**
     * @copydoc publish(geometry_msgs::Point)
     *
     * @param topic Topic where the message should be published
     */
    void publish(geometry_msgs::Point point, const char* topic) { publish(point, topic, "/map"); }
    /**
     * @brief Publish a marker representing a single point
     *
     * @param point Marker location
     */
    void publish(geometry_msgs::Point point) {publish(point, "Point");}

    /**
     * @copydoc publishRel(geometry_msgs::Point)
     * @param topic Topic to be used
     */
    void publishRel(geometry_msgs::Point point, const char* topic) { publish(point, "Point", topic);}
    /**
     * @brief Publishes a single point
     *
     *	Such point is considered relative to the robot frame
     *
     * @param point Point to be published
     */
    void publishRel(geometry_msgs::Point point) { publishRel(point, "/base_footprint");}


    /**
     * @brief Sets the internal ros::NodeHandle
     *
     * @param n_in Handler to be used
     */
    void setNh(ros::NodeHandle n_in);

    /**
     * @copydoc add(const char*,const char*)
     *
     * @param type Type of marker to be added
     */
    void add(const char* name, const char* topic, int type);
    /**
     * @brief Adds a new publicable object
     *
     * @param name The name to reference the publicable object
     * @param topic Topic to be advertised
     */
    void add(const char* name, const char* topic) { add(name, topic, visualization_msgs::Marker::SPHERE);}

    /**
     * @brief Publishes a publicable object
     *
     * @param name Name of the publicable object
     * @param publicable Publicable message
     */
    template <typename T>
    void publish(const char* name, T publicable)
    {
	for(std::vector<pub>::iterator it = pubs.begin(); it != pubs.end(); ++it){
	    if (it->is(name)){
		it->publish(publicable);
		return;
	    }
	}
    }


    /**
     * @brief Set a certain property for a publicable object
     *
     * @param name Name of the publicable object
     * @param property Property to be changed
     */
    template <typename T>
    void setProperty(const char* name, T property){
	for(std::vector<pub>::iterator it = pubs.begin(); it != pubs.end(); ++it){
	    if (it->is(name)){
		it->setProperty(property);
		return;
	    }
	}
    }



private:
    ros::NodeHandle n;			///< Internal node handle to advertise publications
    visualization_msgs::Marker marker;	///< Default marker for direct publications
    static std::vector<pub> pubs;	///< Collection of publicable objects stored to facilitate marker publication

    /**
     * @brief Initial procedure. Sets up the default marker.
     */
    void initialise();
};


} // namespace cam_exploration


#endif

/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <atomic>
#include <tuple>
#include <vector>

#include <aruco_pose/MarkerArray.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std;

sensor_msgs::NavSatFix global_position;
geometry_msgs::PoseStamped local_position;
aruco_pose::MarkerArray marker_array;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

template<typename T, T& STORAGE>
void handleMessage(const T& msg)
{
    STORAGE = msg;
}

const int TARGET_ID = 77;
std::atomic<bool> markers_lock(false);
bool target_found = false;
geometry_msgs::PoseStamped target_pos;
void handleGetAcceptMarker(const aruco_pose::MarkerArray& markerArray) {
    if (std::atomic_exchange(&markers_lock, true)) {
        return;
    }
    target_found = false;
    target_pos.header = markerArray.header;
    stringstream ss;
    ss << "{";
    for (const auto& marker : markerArray.markers) {
        ss << marker.id << ", ";
        if (marker.id != TARGET_ID) {
            continue;
        }
        target_pos.pose = marker.pose;
        target_found = true;
        ROS_INFO("FOUND");
        break;
    }
//    ROS_INFO("xfguo: found markers: %s}", ss.str().c_str());
    std::atomic_store(&markers_lock, false);
}

inline double hypot(double x, double y, double z)
{
    return std::sqrt(x * x + y * y + z * z);
}

inline float getDistance(const geometry_msgs::Point& from, const geometry_msgs::Point& to)
{
    return hypot(from.x - to.x, from.y - to.y, from.z - to.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    auto global_position_sub = nh.subscribe(
            "mavros/global_position/global", 1,
            &handleMessage<sensor_msgs::NavSatFix, global_position>);
    auto local_position_sub = nh.subscribe(
            "mavros/local_position/pose", 1,
            &handleMessage<geometry_msgs::PoseStamped, local_position>);
    auto markers_sub = nh.subscribe("/aruco_detect/markers", 1, &handleGetAcceptMarker);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped ps;
//    ps.header.stamp = ros::Time::now();
    {
        char buf[50];
        sprintf(buf, "aruco_%02d", TARGET_ID);
        ps.header.frame_id = string(buf);
        ROS_INFO("xfguo: frame_id = '%s'", ps.header.frame_id.c_str());
    }
    ps.pose.position.x = 0;
    ps.pose.position.y = 0;
    ps.pose.position.z = 2;
    ps.pose.orientation.x = 1e-10;
    ps.pose.orientation.y = 1e-10;
    ps.pose.orientation.z = 1e-10;
    ps.pose.orientation.w = 1;
    geometry_msgs::PoseStamped pos = tf_buffer.transform(ps, "map");
//    pos.pose.orientation.x = pos.pose.orientation.y = pos.pose.orientation.z = 0;
//    pos.pose.orientation.w = 1;
    ROS_INFO_STREAM("from:\n" << ps << " map to:\n" << pos);

    //send a few setpoints before starting
    ROS_INFO("xfguo: before starting");
    for(int i = 3; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pos);
        ROS_INFO("xfguo: i = %d", i);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("xfguo: ready to start");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    ROS_INFO("A: marker_sub is valid: %d, topic = '%s', pubNum = %d", markers_sub.operator void *(),
             markers_sub.getTopic().c_str(), markers_sub.getNumPublishers() );
    while(ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pos);
        ros::spinOnce();
        rate.sleep();

        if (target_found) {
            break;
        }
    }

    ROS_INFO("Found the target");
    while (ros::ok()) {

        // If found the target, adjust to the goal.
        bool found = false;
        static int count = 0;
        count++;
        if (!atomic_exchange(&markers_lock, true)) {  // lock
            pos.header.frame_id = target_pos.header.frame_id;
            pos.header.stamp = target_pos.header.stamp;
            // Content
            if (target_found) {
                pos.pose = target_pos.pose;
                found = target_found;
            }

            // unlock
            atomic_store(&markers_lock, false);
        }

        if (!found) {
            continue;
        }

        pos.pose.position.z += 1.0;
        geometry_msgs::PoseStamped ps = tf_buffer.transform(pos, "map");
        ROS_INFO_COND(count % 100 == 13, "camera pos: (%f, %f, %f) => map pos: (%f, %f, %f)",
                      pos.pose.position.x, pos.pose.position.y, pos.pose.position.z,
                      ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);

        local_pos_pub.publish(ps);
    }

    return 0;
}

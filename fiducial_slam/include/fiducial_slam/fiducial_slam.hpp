#ifndef FIDUCIAL_SLAM_NODE_H
#define FIDUCIAL_SLAM_NODE_H

#include "rclcpp/rclcpp.hpp"

#include "fiducial_slam/helpers.h"

#include <assert.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "fiducial_msgs/msg/fiducial.hpp"
#include "fiducial_msgs/msg/fiducial_array.hpp"
#include "fiducial_msgs/msg/fiducial_transform.hpp"
#include "fiducial_msgs/msg/fiducial_transform_array.hpp"

#include "fiducial_slam/map.hpp"
#include "fiducial_slam/observation.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include <std_srvs/srv/empty.hpp>
#include "fiducial_msgs/srv/add_fiducial.hpp"

#include <list>
#include <string>

using namespace std;
using namespace cv;


namespace fiducial_slam
{

    class FiducialSlamNode: public rclcpp::Node {
    private:

        bool use_fiducial_area_as_weight;
        double weighting_scale;

        //subscribers
        rclcpp::Subscription<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr msg_sub_tf_arr;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clearSrv;
        rclcpp::Service<fiducial_msgs::srv::AddFiducial>::SharedPtr addFidSrv;

        bool clearMap(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res);
        bool addFiducial(const fiducial_msgs::srv::AddFiducial::Request::SharedPtr req, fiducial_msgs::srv::AddFiducial::Response::SharedPtr res);

        void transformCallback(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg);

        bool declareParams();
        bool getParams();
    public:
        Map fiducialMap;
        FiducialSlamNode();
        ~FiducialSlamNode();
    };

}
#endif //FIDUCIAL_SLAM_NODE_H
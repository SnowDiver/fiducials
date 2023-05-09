#include "../include/fiducial_slam/fiducial_slam.hpp"

using namespace std;
using namespace cv;

namespace fiducial_slam
{

    FiducialSlamNode::FiducialSlamNode() : Node("fiducial_slam_node") {
        
        // Get the parameters
        this->declareParams();
        this->getParams();

        msg_sub_tf_arr = this->create_subscription<fiducial_msgs::msg::FiducialTransformArray>(
            "/aruco_detect/fiducial_transforms", 10,  std::bind(&FiducialSlamNode::transformCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Fiducial Slam ready");
    }

    void FiducialSlamNode::transformCallback(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg) {
        vector<Observation> observations;

        for (size_t i = 0; i < msg->transforms.size(); i++) {
            const fiducial_msgs::msg::FiducialTransform &ft = msg->transforms[i];

            tf2::Vector3 tvec(ft.transform.translation.x, ft.transform.translation.y,
                            ft.transform.translation.z);

            tf2::Quaternion q(ft.transform.rotation.x, ft.transform.rotation.y, ft.transform.rotation.z,
                            ft.transform.rotation.w);

            double variance;
            if (use_fiducial_area_as_weight) {
                variance = weighting_scale / ft.fiducial_area;
            } else {
                variance = weighting_scale * ft.object_error;
            }

            Observation obs(ft.fiducial_id, tf2::Stamped<TransformWithVariance>(
                                                TransformWithVariance(ft.transform, variance),
                                                tf2_ros::fromMsg(msg->header.stamp), msg->header.frame_id));
            observations.push_back(obs);
        }

        fiducialMap.update(observations, msg->header.stamp);
    }

    /**
     * Declares ros parameters and their standard value.
     * @return bool true
     */
    bool FiducialSlamNode::declareParams() {
        // If set, use the fiducial area in pixels^2 as an indication of the
        // 'goodness' of it. This will favor fiducials that are close to the
        // camera and center of the image. The reciprical of the area is actually
        // used, in place of reprojection error as the estimate's variance
        this->declare_parameter("use_fiducial_area_as_weight", false);
        // Scaling factor for weighing
        this->declare_parameter("weighting_scale", 1e9);
        return true;
    }

    /**
     * Gets the ros parameters and places them in a variable.
     * @return bool true
     */
    bool FiducialSlamNode::getParams() {
        this->get_parameter("use_fiducial_area_as_weight", use_fiducial_area_as_weight);
        this->get_parameter("weighting_scale", weighting_scale);
        return true;
    }

    FiducialSlamNode::~FiducialSlamNode() { /** */ }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fiducial_slam::FiducialSlamNode>());
    rclcpp::shutdown();
    return 0;
}
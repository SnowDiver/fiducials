/*
 * Copyright (c) 2017-20, Ubiquity Robotics Inc., Austin Hendrix
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#include <assert.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <visualization_msgs/msg/marker.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
//#include <dynamic_reconfigure/server.h>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "fiducial_msgs/msg/fiducial.hpp"
#include "fiducial_msgs/msg/fiducial_array.hpp"
#include "fiducial_msgs/msg/fiducial_transform.hpp"
#include "fiducial_msgs/msg/fiducial_transform_array.hpp"
//#include "aruco_detect/DetectorParamsConfig.h"

#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <list>
#include <string>
#include <aruco_detect/helpers.h>

using namespace std;
using namespace cv;

class FiducialsNode: public rclcpp::Node
{
  private:
    rclcpp::Publisher<fiducial_msgs::msg::FiducialArray>::SharedPtr vertices_pub;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pose_pub_vision;
    rclcpp::Publisher<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr pose_pub_fiducial;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub;
    rclcpp::Subscription<fiducial_msgs::msg::FiducialArray>::SharedPtr vertices_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ignore_sub;
    std::shared_ptr<image_transport::ImageTransport> img_trans;
    image_transport::Subscriber img_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_enable_detections;

    // if set, we publish the images that contain fiducials
    bool publish_images;
    bool enable_detections;
    bool vis_msgs;

    std::string topic_sub_cam_image;
    std::string topic_sub_cam_info;

    bool intrinsics_override_enable;
    float fov_h;
    int w;
    int h;

    double fiducial_len;

    bool doPoseEstimation;
    bool haveCamInfo;
    bool publishFiducialTf;
    vector <vector <Point2f> > corners;
    vector <int> ids;
    cv_bridge::CvImagePtr cv_ptr;

    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    int frameNum;
    std::string frameId;
    std::vector<int> ignoreIds;
    std::map<int, double> fiducialLens;

private:
    image_transport::Publisher image_pub;

    cv::Ptr<aruco::DetectorParameters> detectorParams;
    cv::Ptr<aruco::Dictionary> dictionary;

    void handleIgnoreString(const std::string& str);

    void estimatePoseSingleMarkers(float markerLength,
                                   const cv::Mat &cameraMatrix,
                                   const cv::Mat &distCoeffs,
                                   vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                   vector<double>& reprojectionError);


    void ignoreCallback(const std_msgs::msg::String::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void poseEstimateCallback(const fiducial_msgs::msg::FiducialArray::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult param_change_callback(const std::vector<rclcpp::Parameter> & parameters);

    bool enableDetectionsCallback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                                  std_srvs::srv::SetBool::Response::SharedPtr res);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle;

  public:
    FiducialsNode();
};


/**
  * @brief Return object points for the system centered in a single marker, given the marker length
  */
static void getSingleMarkerObjectPoints(float markerLength, vector<Point3f>& objPoints) {

    CV_Assert(markerLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.clear();
    objPoints.push_back(Vec3f(-markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f,-markerLength / 2.f, 0));
    objPoints.push_back(Vec3f(-markerLength / 2.f,-markerLength / 2.f, 0));
}

// Euclidean distance between two points
static double dist(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}

// Compute area in image of a fiducial, using Heron's formula
// to find the area of two triangles
static double calcFiducialArea(const std::vector<cv::Point2f> &pts)
{
    const Point2f &p0 = pts.at(0);
    const Point2f &p1 = pts.at(1);
    const Point2f &p2 = pts.at(2);
    const Point2f &p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
    a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
    return a1+a2;
}

// estimate reprojection error
static double getReprojectionError(const vector<Point3f> &objectPoints,
                            const vector<Point2f> &imagePoints,
                            const Mat &cameraMatrix, const Mat  &distCoeffs,
                            const Vec3d &rvec, const Vec3d &tvec) {

    vector<Point2f> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix,
                      distCoeffs, projectedPoints);

    // calculate RMS image error
    double totalError = 0.0;
    for (unsigned int i=0; i<objectPoints.size(); i++) {
        double error = dist(imagePoints[i], projectedPoints[i]);
        totalError += error*error;
    }
    double rerror = totalError/(double)objectPoints.size();
    return rerror;
}

void FiducialsNode::estimatePoseSingleMarkers(float markerLength,
                                const cv::Mat &cameraMatrix,
                                const cv::Mat &distCoeffs,
                                vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                vector<double>& reprojectionError) {

    CV_Assert(markerLength > 0);

    vector<Point3f> markerObjPoints;
    int nMarkers = (int)corners.size();
    rvecs.reserve(nMarkers);
    tvecs.reserve(nMarkers);
    reprojectionError.reserve(nMarkers);

    // for each marker, calculate its pose
    for (int i = 0; i < nMarkers; i++) {
       double fiducialSize = markerLength;

       std::map<int, double>::iterator it = fiducialLens.find(ids[i]);
       if (it != fiducialLens.end()) {
          fiducialSize = it->second;
       }

       getSingleMarkerObjectPoints(fiducialSize, markerObjPoints);
       cv::solvePnP(markerObjPoints, corners[i], cameraMatrix, distCoeffs,
                    rvecs[i], tvecs[i]);

       reprojectionError[i] =
          getReprojectionError(markerObjPoints, corners[i],
                               cameraMatrix, distCoeffs,
                               rvecs[i], tvecs[i]);
    }
}

rcl_interfaces::msg::SetParametersResult
FiducialsNode::param_change_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_WARN(this->get_logger(), "Param set");
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    // RCLCPP_INFO(this->get_logger(), "Parameter '%s' changed.", parameter.get_name().c_str());
  }

  try
  {
    detectorParams->adaptiveThreshConstant = this->get_parameter("adaptiveThreshConstant").as_double();
    detectorParams->adaptiveThreshWinSizeMin = this->get_parameter("adaptiveThreshWinSizeMin").as_int();
    detectorParams->adaptiveThreshWinSizeMax = this->get_parameter("adaptiveThreshWinSizeMax").as_int();
    detectorParams->adaptiveThreshWinSizeStep = this->get_parameter("adaptiveThreshWinSizeStep").as_int();
    detectorParams->cornerRefinementMaxIterations = this->get_parameter("cornerRefinementMaxIterations").as_int();
    detectorParams->cornerRefinementMinAccuracy = this->get_parameter("cornerRefinementMinAccuracy").as_double();
    detectorParams->cornerRefinementWinSize = this->get_parameter("cornerRefinementWinSize").as_int();
    #if CV_MINOR_VERSION==2 and CV_MAJOR_VERSION==3
        detectorParams->doCornerRefinement = this->get_parameter("doCornerRefinement").as_bool();
    #else
        bool doCornerRefinement = this->get_parameter("doCornerRefinement").as_bool();
        if (doCornerRefinement) {
            bool cornerRefinementSubpix = this->get_parameter("cornerRefinementSubpix").as_bool();
        if (cornerRefinementSubpix) {
            detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
        }
        else {
            detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
        }
        }
        else {
        detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_NONE;
        }
    #endif

    detectorParams->errorCorrectionRate = this->get_parameter("errorCorrectionRate").as_double();
    detectorParams->minCornerDistanceRate = this->get_parameter("minCornerDistanceRate").as_double();
    detectorParams->markerBorderBits = this->get_parameter("markerBorderBits").as_int();
    detectorParams->maxErroneousBitsInBorderRate = this->get_parameter("maxErroneousBitsInBorderRate").as_double();
    detectorParams->minDistanceToBorder = this->get_parameter("minDistanceToBorder").as_int();
    detectorParams->minMarkerDistanceRate = this->get_parameter("minMarkerDistanceRate").as_double();
    detectorParams->minMarkerPerimeterRate = this->get_parameter("minMarkerPerimeterRate").as_double();
    detectorParams->maxMarkerPerimeterRate = this->get_parameter("maxMarkerPerimeterRate").as_double();
    detectorParams->minOtsuStdDev = this->get_parameter("minOtsuStdDev").as_double();
    detectorParams->perspectiveRemoveIgnoredMarginPerCell = this->get_parameter("perspectiveRemoveIgnoredMarginPerCell").as_double();
    detectorParams->perspectiveRemovePixelPerCell = this->get_parameter("perspectiveRemovePixelPerCell").as_int();
    detectorParams->polygonalApproxAccuracyRate = this->get_parameter("polygonalApproxAccuracyRate").as_double();

  }
  catch(const rclcpp::exceptions::ParameterNotDeclaredException & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not update parameter. %s", e.what());
    result.successful = false;
    return result;
  }

  return result;
}

void FiducialsNode::ignoreCallback(const std_msgs::msg::String::SharedPtr msg)
{
    ignoreIds.clear();
    this->set_parameter(rclcpp::Parameter("ignore_fiducials", msg->data));
    handleIgnoreString(msg->data);
}

void FiducialsNode::camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (haveCamInfo) {
        return;
    }

    if (msg->k != std::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        

        // This overrides the camera_info msg, 
        // this is needed when using ignition gazebo as the cam info from the sim is incorrect.
        if(intrinsics_override_enable) {
            float fx, fy, cx, cy;
            cx = 0.5 * w;
            cy = 0.5 * h;
            fx = w / (2.0 * std::tan(fov_h / 2.0));
            fy = fx;  //h/(2.0*std::tan(fov_h/2.0));
            
            msg->k[0] = fx;
            msg->k[2] = cx;
            msg->k[4] = fy;
            msg->k[5] = cy;
            msg->k[8] = 1.0;

            msg->p[0] = fx;
            msg->p[2] = cx;
            msg->p[5] = fy;
            msg->p[6] = cy;
            msg->p[10] = 1.0;
        }
        
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                cameraMatrix.at<double>(i, j) = msg->k[i*3+j];
            }
        }

        for (int i=0; i<5; i++) {
            distortionCoeffs.at<double>(0,i) = msg->d[i];
        }

        haveCamInfo = true;
        frameId = msg->header.frame_id;
    }
    else {
        RCLCPP_WARN(this->get_logger(), "%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
    }
}

void FiducialsNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    if (enable_detections == false) {
        return; //return without doing anything
    }

    // RCLCPP_INFO(this->get_logger(), "Got image");

    fiducial_msgs::msg::FiducialArray fva;
    fva.header.stamp = msg->header.stamp;
    fva.header.frame_id = frameId;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams);
        // RCLCPP_INFO(this->get_logger(), "Detected %d markers", (int)ids.size());

        for (size_t i=0; i<ids.size(); i++) {
	    if (std::count(ignoreIds.begin(), ignoreIds.end(), ids[i]) != 0) {
	        // RCLCPP_INFO(this->get_logger(), "Ignoring id %d", ids[i]);
	        continue;
	    }
            fiducial_msgs::msg::Fiducial fid;
            fid.fiducial_id = ids[i];

            fid.x0 = corners[i][0].x;
            fid.y0 = corners[i][0].y;
            fid.x1 = corners[i][1].x;
            fid.y1 = corners[i][1].y;
            fid.x2 = corners[i][2].x;
            fid.y2 = corners[i][2].y;
            fid.x3 = corners[i][3].x;
            fid.y3 = corners[i][3].y;
            fva.fiducials.push_back(fid);
        }

        vertices_pub->publish(fva);

        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        }

        if (publish_images) {
	    image_pub.publish(cv_ptr->toImageMsg());
        }
    }
    catch(cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch(cv::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv exception: %s", e.what());
    }
}

void FiducialsNode::poseEstimateCallback(const fiducial_msgs::msg::FiducialArray::SharedPtr msg)
{
    vector <Vec3d>  rvecs, tvecs;

    vision_msgs::msg::Detection2DArray vma;
    fiducial_msgs::msg::FiducialTransformArray fta;
    if (vis_msgs) {
	    vma.header.stamp = msg->header.stamp;
	    vma.header.frame_id = frameId;
    }
    else {
	    fta.header.stamp = msg->header.stamp;
    	fta.header.frame_id = frameId;
    }
    frameNum++;

    if (doPoseEstimation) {
        try {
            if (!haveCamInfo) {
                if (frameNum > 5) {
                    RCLCPP_ERROR(this->get_logger(),"No camera intrinsics");
                }
                return;
            }

            vector <double>reprojectionError;
            estimatePoseSingleMarkers((float)fiducial_len,
                                      cameraMatrix, distortionCoeffs,
                                      rvecs, tvecs,
                                      reprojectionError);

            for (size_t i=0; i<ids.size(); i++) {
                aruco::drawAxis(cv_ptr->image, cameraMatrix, distortionCoeffs,
                                rvecs[i], tvecs[i], (float)fiducial_len);

                // RCLCPP_INFO(this->get_logger(), "Detected id %d T %.2f %.2f %.2f R %.2f %.2f %.2f", ids[i],
                        //  tvecs[i][0], tvecs[i][1], tvecs[i][2],
                        //  rvecs[i][0], rvecs[i][1], rvecs[i][2]);

                if (std::count(ignoreIds.begin(), ignoreIds.end(), ids[i]) != 0) {
                    // RCLCPP_INFO(this->get_logger(), "Ignoring id %d", ids[i]);
                    continue;
                }

                double angle = norm(rvecs[i]);
                Vec3d axis = rvecs[i] / angle;
                // RCLCPP_INFO(this->get_logger(), "angle %f axis %f %f %f",
                        //  angle, axis[0], axis[1], axis[2]);

                double object_error =
                    (reprojectionError[i] / dist(corners[i][0], corners[i][2])) *
                    (norm(tvecs[i]) / fiducial_len);

                // Standard ROS vision_msgs
                fiducial_msgs::msg::FiducialTransform ft;
                tf2::Quaternion q;
                if (vis_msgs) {
                    vision_msgs::msg::Detection2D vm;
                    vision_msgs::msg::ObjectHypothesisWithPose vmh;
                    vmh.hypothesis.class_id = ids[i];
                    vmh.hypothesis.score = exp(-2 * object_error); // [0, infinity] -> [1,0]
                        vmh.pose.pose.position.x = tvecs[i][0];
                    vmh.pose.pose.position.y = tvecs[i][1];
                    vmh.pose.pose.position.z = tvecs[i][2];
                    q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
                    vmh.pose.pose.orientation.w = q.w();
                    vmh.pose.pose.orientation.x = q.x();
                    vmh.pose.pose.orientation.y = q.y();
                    vmh.pose.pose.orientation.z = q.z();

                    vm.results.push_back(vmh);
                    vma.detections.push_back(vm);
                }
                else {
                    ft.fiducial_id = ids[i];

                    ft.transform.translation.x = tvecs[i][0];
                    ft.transform.translation.y = tvecs[i][1];
                    ft.transform.translation.z = tvecs[i][2];
                    q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
                    ft.transform.rotation.w = q.w();
                    ft.transform.rotation.x = q.x();
                    ft.transform.rotation.y = q.y();
                    ft.transform.rotation.z = q.z();
                    ft.fiducial_area = calcFiducialArea(corners[i]);
                    ft.image_error = reprojectionError[i];
                    // Convert image_error (in pixels) to object_error (in meters)
                    ft.object_error =
                        (reprojectionError[i] / dist(corners[i][0], corners[i][2])) *
                        (norm(tvecs[i]) / fiducial_len);

                    // if (ft.transform.translation.z > 6 || ft.transform.translation.x > 6) {
                    //      RCLCPP_INFO(this->get_logger(), "Marker not added to TFA, to far away: (%s, %s)", std::to_string(ft.transform.translation.z).c_str(), std::to_string(ft.transform.translation.x).c_str());
                    //     continue;
                    // }

                    fta.transforms.push_back(ft);
                }

                // Publish tf for the fiducial relative to the camera
                if (publishFiducialTf) {
                    if (vis_msgs) {
                                geometry_msgs::msg::TransformStamped ts;
                                ts.transform.translation.x = tvecs[i][0];
                                ts.transform.translation.y = tvecs[i][1];
                                ts.transform.translation.z = tvecs[i][2];
                                ts.transform.rotation.w = q.w();
                                ts.transform.rotation.x = q.x();
                                ts.transform.rotation.y = q.y();
                                ts.transform.rotation.z = q.z();
                                ts.header.frame_id = frameId;
                                ts.header.stamp = msg->header.stamp;
                                ts.child_frame_id = "fiducial_" + std::to_string(ids[i]);
                                broadcaster->sendTransform(ts);
                    }
                    else {
                    geometry_msgs::msg::TransformStamped ts;
                                ts.transform = ft.transform;
                                ts.header.frame_id = frameId;
                                ts.header.stamp = msg->header.stamp;
                                ts.child_frame_id = "fiducial_" + std::to_string(ft.fiducial_id);
                                broadcaster->sendTransform(ts);
                    }
                }
            }
        }
        catch(cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
        catch(cv::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv exception: %s", e.what());
        }
    }
    if (vis_msgs)
    	pose_pub_vision->publish(vma);
    else 
	pose_pub_fiducial->publish(fta);
}

void FiducialsNode::handleIgnoreString(const std::string& str)
{
    RCLCPP_INFO(this->get_logger(), "Ignoring fiducials %s ", str.c_str());

    /*
    ignogre fiducials can take comma separated list of individual
    fiducial ids or ranges, eg "1,4,8,9-12,30-40"
    */
    std::vector<std::string> strs;
    split(strs, str, ',');
    for (const string& element : strs) {
        if (element == "") {
           continue;
        }
        std::vector<std::string> range;
        split(range, element, '-');
        if (range.size() == 2) {
           int start = std::stoi(range[0]);
           int end = std::stoi(range[1]);
           RCLCPP_INFO(this->get_logger(), "Ignoring fiducial id range %d to %d", start, end);
           for (int j=start; j<=end; j++) {
               ignoreIds.push_back(j);
           }
        }
        else if (range.size() == 1) {
           int fid = std::stoi(range[0]);
           RCLCPP_INFO(this->get_logger(), "Ignoring fiducial id %d", fid);
           ignoreIds.push_back(fid);
        }
        else {
           RCLCPP_ERROR(this->get_logger(), "Malformed ignore_fiducials: %s", element.c_str());
        }
    }
}

bool FiducialsNode::enableDetectionsCallback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                                             std_srvs::srv::SetBool::Response::SharedPtr res)
{
    enable_detections = req->data;
    if (enable_detections){
        res->message = "Enabled aruco detections.";
        // RCLCPP_INFO(this->get_logger(), "Enabled aruco detections.");
    }
    else {
        res->message = "Disabled aruco detections.";
        // RCLCPP_INFO(this->get_logger(), "Disabled aruco detections.");
    }
    
    res->success = true;
    return true;
}


FiducialsNode::FiducialsNode() : Node("fiducials_node")
{
    std::cerr << "CTOR\n";

    // RCLCPP_INFO(this->get_logger(), "Start");

    broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // img_trans = std::make_shared<image_transport::ImageTransport>(this);

    frameNum = 0;

    // Camera intrinsics
    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

    // distortion coefficients
    distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    haveCamInfo = false;
    enable_detections = true;

    int dicno;

    detectorParams = new aruco::DetectorParameters();

    publish_images = this->declare_parameter("publish_images", false);
    fiducial_len = this->declare_parameter("fiducial_len", 0.14);
    dicno = this->declare_parameter("dictionary", 7);
    doPoseEstimation = this->declare_parameter("do_pose_estimation", true);
    publishFiducialTf = this->declare_parameter("publish_fiducial_tf", true);
    vis_msgs = this->declare_parameter("vis_msgs", false);

    topic_sub_cam_image = this->declare_parameter("topic_sub_cam_image", "/camera/image");
    topic_sub_cam_info = this->declare_parameter("topic_sub_cam_info", "/camera/camera_info");

    intrinsics_override_enable = this->declare_parameter("intrinsics_override.enable", false);
    fov_h = this->declare_parameter("intrinsics_override.fov_h", 0.9799);
    w = this->declare_parameter("intrinsics_override.w", 1920);
    h = this->declare_parameter("intrinsics_override.h", 1080);

    std::string str;
    std::vector<std::string> strs;

    str = this->declare_parameter("ignore_fiducials", std::string());
    handleIgnoreString(str);

    /*
    fiducial size can take comma separated list of size: id or size: range,
    e.g. "200.0: 12, 300.0: 200-300"
    */
    str = this->declare_parameter("fiducial_len_override", std::string());
    split(strs, str, ',');
    for (const string& element : strs) {
        if (element == "") {
           continue;
        }
        std::vector<std::string> parts;
        split(parts, element, ':');
        if (parts.size() == 2) {
            double len = std::stod(parts[1]);
            std::vector<std::string> range;
            split(range, element, '-');
            if (range.size() == 2) {
               int start = std::stoi(range[0]);
               int end = std::stoi(range[1]);
            //    // RCLCPP_INFO(this->get_logger(), "Setting fiducial id range %d - %d length to %f", start, end, len);
               for (int j=start; j<=end; j++) {
                   fiducialLens[j] = len;
               }
            }
            else if (range.size() == 1){
               int fid = std::stoi(range[0]);
            //    // RCLCPP_INFO(this->get_logger(), "Setting fiducial id %d length to %f", fid, len);
               fiducialLens[fid] = len;
            }
            else {
               RCLCPP_ERROR(this->get_logger(), "Malformed fiducial_len_override: %s", element.c_str());
            }
        }
        else {
           RCLCPP_ERROR(this->get_logger(), "Malformed fiducial_len_override: %s", element.c_str());
        }
    }

    // image_pub = img_trans->advertise("/fiducial_images", 1);
    image_pub = image_transport::create_publisher(this, "/fiducial_images");

    vertices_pub = this->create_publisher<fiducial_msgs::msg::FiducialArray>("fiducial_vertices", 1);

    if (vis_msgs)
    	pose_pub_vision = this->create_publisher<vision_msgs::msg::Detection2DArray>("fiducial_transforms", 1);
    else
	pose_pub_fiducial = this->create_publisher<fiducial_msgs::msg::FiducialTransformArray>("fiducial_transforms", 1);

    dictionary = aruco::getPredefinedDictionary(dicno);
    
    image_transport::TransportHints hints(this, "raw");

    // img_sub = img_trans->subscribe("/camera/image", 1,
    //                        &FiducialsNode::imageCallback, this);

    img_sub = image_transport::create_subscription(
    this, topic_sub_cam_image, std::bind(&FiducialsNode::imageCallback, this, std::placeholders::_1), hints.getTransport());

    vertices_sub = this->create_subscription<fiducial_msgs::msg::FiducialArray>("fiducial_vertices", 1,
                     std::bind(&FiducialsNode::poseEstimateCallback, this, std::placeholders::_1));

    caminfo_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(topic_sub_cam_info, 1,
                     std::bind(&FiducialsNode::camInfoCallback, this, std::placeholders::_1));

    ignore_sub = this->create_subscription<std_msgs::msg::String>("ignore_fiducials", 1,
                                         std::bind(&FiducialsNode::ignoreCallback, this, std::placeholders::_1));

    service_enable_detections = this->create_service<std_srvs::srv::SetBool>("enable_detections", std::bind(&FiducialsNode::enableDetectionsCallback, this, std::placeholders::_1, std::placeholders::_2));

    /* adaptiveThreshConstant */
    auto adaptiveThreshConstantDescription = rcl_interfaces::msg::ParameterDescriptor{};
    adaptiveThreshConstantDescription.name = "Constant for adaptive thresholding before finding contours";
    auto adaptiveThreshConstantRange = rcl_interfaces::msg::FloatingPointRange{};
    adaptiveThreshConstantRange.from_value = 0;
    adaptiveThreshConstantRange.to_value = std::numeric_limits<double>::infinity();
    adaptiveThreshConstantDescription.floating_point_range = {adaptiveThreshConstantRange};
    detectorParams->adaptiveThreshConstant = this->declare_parameter("adaptiveThreshConstant", 7.0, adaptiveThreshConstantDescription);

    /* adaptiveThreshWinSizeMax */
    auto adaptiveThreshWinSizeMaxDescription = rcl_interfaces::msg::ParameterDescriptor{};
    adaptiveThreshWinSizeMaxDescription.name = "Maximum window size for adaptive thresholding before finding contours";
    auto adaptiveThreshWinSizeMaxRange = rcl_interfaces::msg::IntegerRange();
    adaptiveThreshWinSizeMaxRange.from_value = 1;
    adaptiveThreshWinSizeMaxRange.to_value = std::numeric_limits<int>::max();
    adaptiveThreshWinSizeMaxDescription.integer_range = {adaptiveThreshWinSizeMaxRange};
    detectorParams->adaptiveThreshWinSizeMax = this->declare_parameter("adaptiveThreshWinSizeMax", 53, adaptiveThreshWinSizeMaxDescription); /* default 23 */

    /* adaptiveThreshWinSizeMin */
    auto adaptiveThreshWinSizeMinDescription = rcl_interfaces::msg::ParameterDescriptor{};
    adaptiveThreshWinSizeMinDescription.name = "Minimum window size for adaptive thresholding before finding contours";
    auto adaptiveThreshWinSizeMinRange = rcl_interfaces::msg::IntegerRange();
    adaptiveThreshWinSizeMinRange.from_value = 1;
    adaptiveThreshWinSizeMinRange.to_value = std::numeric_limits<int>::max();
    adaptiveThreshWinSizeMinDescription.integer_range = {adaptiveThreshWinSizeMinRange};
    detectorParams->adaptiveThreshWinSizeMin = this->declare_parameter("adaptiveThreshWinSizeMin", 3, adaptiveThreshWinSizeMinDescription);

    /* adaptiveThreshWinSizeStep */
    auto adaptiveThreshWinSizeStepDescription = rcl_interfaces::msg::ParameterDescriptor{};
    adaptiveThreshWinSizeStepDescription.name = "Increments from adaptiveThreshWinSizeMin to adaptiveThreshWinSizeMax during the thresholding";
    auto adaptiveThreshWinSizeStepRange = rcl_interfaces::msg::IntegerRange();
    adaptiveThreshWinSizeStepRange.from_value = 1;
    adaptiveThreshWinSizeStepRange.to_value = std::numeric_limits<int>::max();
    adaptiveThreshWinSizeStepDescription.integer_range = {adaptiveThreshWinSizeStepRange};
    detectorParams->adaptiveThreshWinSizeStep = this->declare_parameter("adaptiveThreshWinSizeStep", 4, adaptiveThreshWinSizeStepDescription); /* default 10 */

    /* cornerRefinementMaxIterations */
    auto cornerRefinementMaxIterationsDescription = rcl_interfaces::msg::ParameterDescriptor{};
    cornerRefinementMaxIterationsDescription.name = "Maximum number of iterations for stop criteria of the corner refinement process";
    auto cornerRefinementMaxIterationsRange = rcl_interfaces::msg::IntegerRange();
    cornerRefinementMaxIterationsRange.from_value = 1;
    cornerRefinementMaxIterationsRange.to_value = std::numeric_limits<int>::max();
    cornerRefinementMaxIterationsDescription.integer_range = {cornerRefinementMaxIterationsRange};
    detectorParams->cornerRefinementMaxIterations = this->declare_parameter("cornerRefinementMaxIterations", 30, cornerRefinementMaxIterationsDescription);

    /* cornerRefinementMinAccuracy */
    auto cornerRefinementMinAccuracyDescription = rcl_interfaces::msg::ParameterDescriptor{};
    cornerRefinementMinAccuracyDescription.name = "Minimum error for the stop criteria of the corner refinement process";
    auto cornerRefinementMinAccuracyRange = rcl_interfaces::msg::FloatingPointRange();
    cornerRefinementMinAccuracyRange.from_value = 0.0;
    cornerRefinementMinAccuracyRange.to_value = 1.0;
    cornerRefinementMinAccuracyDescription.floating_point_range = {cornerRefinementMinAccuracyRange};
    detectorParams->cornerRefinementMinAccuracy = this->declare_parameter("cornerRefinementMinAccuracy", 0.01, cornerRefinementMinAccuracyDescription); /* default 0.1 */

    /* cornerRefinementWinSize */
    auto cornerRefinementWinSizeDescription = rcl_interfaces::msg::ParameterDescriptor{};
    cornerRefinementWinSizeDescription.name = "Window size for the corner refinement process (in pixels)";
    auto cornerRefinementWinSizeRange = rcl_interfaces::msg::IntegerRange();
    cornerRefinementWinSizeRange.from_value = 1;
    cornerRefinementWinSizeRange.to_value = std::numeric_limits<int>::max();
    cornerRefinementWinSizeDescription.integer_range = {cornerRefinementWinSizeRange};
    detectorParams->cornerRefinementWinSize = this->declare_parameter("cornerRefinementWinSize", 5, cornerRefinementWinSizeDescription);

#if CV_MINOR_VERSION==2 and CV_MAJOR_VERSION==3
    detectorParams->doCornerRefinement = this->declare_parameter("doCornerRefinement", true); /* default false */
#else
    bool doCornerRefinement = true;

    auto doCornerRefinementDescription = rcl_interfaces::msg::ParameterDescriptor{};
    doCornerRefinementDescription.name = "Whether to do subpixel corner refinement";
    doCornerRefinement = this->declare_parameter("doCornerRefinement", true, doCornerRefinementDescription);
    if (doCornerRefinement) {
       bool cornerRefinementSubpix = true;
       auto cornerRefinementSubpixDescription = rcl_interfaces::msg::ParameterDescriptor{};
       cornerRefinementSubpixDescription.name = "Whether to do subpixel corner refinement (true) or contour (false)";
       cornerRefinementSubpix = this->declare_parameter("cornerRefinementSubpix", true, cornerRefinementSubpixDescription);
       if (cornerRefinementSubpix) {
         detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
       }
       else {
         detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
       }
    }
    else {
       detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_NONE;
    }
#endif


    /* errorCorrectionRate */
    auto errorCorrectionRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    errorCorrectionRateDescription.name = "Error correction rate respect to the maximum error correction capability for each dictionary";
    auto errorCorrectionRateRange = rcl_interfaces::msg::FloatingPointRange();
    errorCorrectionRateRange.from_value = 0.0;
    errorCorrectionRateRange.to_value = 1.0;
    errorCorrectionRateDescription.floating_point_range = {errorCorrectionRateRange};
    detectorParams->errorCorrectionRate = this->declare_parameter("errorCorrectionRate", 0.6, errorCorrectionRateDescription);

    /* minCornerDistanceRate */
    auto minCornerDistanceRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    minCornerDistanceRateDescription.name = "Minimum distance between corners for detected markers relative to its perimeter";
    auto minCornerDistanceRateRange = rcl_interfaces::msg::FloatingPointRange();
    minCornerDistanceRateRange.from_value = 0;
    minCornerDistanceRateRange.to_value = std::numeric_limits<double>::infinity();
    minCornerDistanceRateDescription.floating_point_range = {minCornerDistanceRateRange};
    detectorParams->minCornerDistanceRate = this->declare_parameter("minCornerDistanceRate", 0.05, minCornerDistanceRateDescription);

    /* markerBorderBits */
    auto markerBorderBitsDescription = rcl_interfaces::msg::ParameterDescriptor{};
    markerBorderBitsDescription.name = "Number of bits of the marker border, i.e. marker border width";
    auto markerBorderBitsRange = rcl_interfaces::msg::IntegerRange();
    markerBorderBitsRange.from_value = 0;
    markerBorderBitsRange.to_value = std::numeric_limits<int>::max();
    markerBorderBitsDescription.integer_range = {markerBorderBitsRange};
    detectorParams->markerBorderBits = this->declare_parameter("markerBorderBits", 1, markerBorderBitsDescription);

    /* maxErroneousBitsInBorderRate */
    auto maxErroneousBitsInBorderRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    maxErroneousBitsInBorderRateDescription.name = "Maximum number of accepted erroneous bits in the border (i.e. number of allowed white bits in the border)";
    auto maxErroneousBitsInBorderRateRange = rcl_interfaces::msg::FloatingPointRange();
    maxErroneousBitsInBorderRateRange.from_value = 0.0;
    maxErroneousBitsInBorderRateRange.to_value = 1.0;
    maxErroneousBitsInBorderRateDescription.floating_point_range = {maxErroneousBitsInBorderRateRange};
    detectorParams->maxErroneousBitsInBorderRate = this->declare_parameter("maxErroneousBitsInBorderRate", 0.04, maxErroneousBitsInBorderRateDescription);

    /* minDistanceToBorder */
    auto minDistanceToBorderDescription = rcl_interfaces::msg::ParameterDescriptor{};
    minDistanceToBorderDescription.name = "Minimum distance of any corner to the image border for detected markers (in pixels)";
    auto minDistanceToBorderRange = rcl_interfaces::msg::IntegerRange();
    minDistanceToBorderRange.from_value = 0;
    minDistanceToBorderRange.to_value = std::numeric_limits<int>::max();
    minDistanceToBorderDescription.integer_range = {minDistanceToBorderRange};
    detectorParams->minDistanceToBorder = this->declare_parameter("minDistanceToBorder", 3, minDistanceToBorderDescription);

    /* minMarkerDistanceRate */
    auto minMarkerDistanceRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    minMarkerDistanceRateDescription.name = "Minimum mean distance beetween two marker corners to be considered similar, so that the smaller one is removed. The rate is relative to the smaller perimeter of the two markers";
    auto minMarkerDistanceRateRange = rcl_interfaces::msg::FloatingPointRange();
    minMarkerDistanceRateRange.from_value = 0.0;
    minMarkerDistanceRateRange.to_value = 1.0;
    minMarkerDistanceRateDescription.floating_point_range = {minMarkerDistanceRateRange};
    detectorParams->minMarkerDistanceRate = this->declare_parameter("minMarkerDistanceRate", 0.05, minMarkerDistanceRateDescription);

    /* minMarkerPerimeterRate */
    auto minMarkerPerimeterRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    minMarkerPerimeterRateDescription.name = "Determine minumum perimeter for marker contour to be detected. This is defined as a rate respect to the maximum dimension of the input image";
    auto minMarkerPerimeterRateRange = rcl_interfaces::msg::FloatingPointRange();
    minMarkerPerimeterRateRange.from_value = 0.0;
    minMarkerPerimeterRateRange.to_value = 1.0;
    minMarkerPerimeterRateDescription.floating_point_range = {minMarkerPerimeterRateRange};
    detectorParams->minMarkerPerimeterRate = this->declare_parameter("minMarkerPerimeterRate", 0.1, minMarkerPerimeterRateDescription); /* default 0.3 */

    /* maxMarkerPerimeterRate */
    auto maxMarkerPerimeterRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    maxMarkerPerimeterRateDescription.name = "Determine maximum perimeter for marker contour to be detected. This is defined as a rate respect to the maximum dimension of the input image";
    auto maxMarkerPerimeterRateRange = rcl_interfaces::msg::IntegerRange();
    maxMarkerPerimeterRateRange.from_value = 0.0;
    maxMarkerPerimeterRateRange.to_value = 1.0;
    maxMarkerPerimeterRateDescription.integer_range = {maxMarkerPerimeterRateRange};
    detectorParams->maxMarkerPerimeterRate = this->declare_parameter("maxMarkerPerimeterRate", 4.0, maxMarkerPerimeterRateDescription);

    /* minOtsuStdDev */
    auto minOtsuStdDevDescription = rcl_interfaces::msg::ParameterDescriptor{};
    minOtsuStdDevDescription.name = "Minimum standard deviation in pixels values during the decodification step to apply Otsu thresholding (otherwise, all the bits are set to 0 or 1 depending on mean higher than 128 or not)";
    auto minOtsuStdDevRange = rcl_interfaces::msg::FloatingPointRange();
    minOtsuStdDevRange.from_value = 0.0;
    minOtsuStdDevRange.to_value = std::numeric_limits<double>::infinity();
    minOtsuStdDevDescription.floating_point_range = {minOtsuStdDevRange};
    detectorParams->minOtsuStdDev = this->declare_parameter("minOtsuStdDev", 5.0, minOtsuStdDevDescription);

    /* perspectiveRemoveIgnoredMarginPerCell */
    auto perspectiveRemoveIgnoredMarginPerCellDescription = rcl_interfaces::msg::ParameterDescriptor{};
    perspectiveRemoveIgnoredMarginPerCellDescription.name = "Width of the margin of pixels on each cell not considered for the determination of the cell bit. Represents the rate respect to the total size of the cell, i.e. perpectiveRemovePixelPerCell";
    auto perspectiveRemoveIgnoredMarginPerCellRange = rcl_interfaces::msg::FloatingPointRange();
    perspectiveRemoveIgnoredMarginPerCellRange.from_value = 0.0;
    perspectiveRemoveIgnoredMarginPerCellRange.to_value = 1.0;
    perspectiveRemoveIgnoredMarginPerCellDescription.floating_point_range = {perspectiveRemoveIgnoredMarginPerCellRange};
    detectorParams->perspectiveRemoveIgnoredMarginPerCell = this->declare_parameter("perspectiveRemoveIgnoredMarginPerCell", 0.13, perspectiveRemoveIgnoredMarginPerCellDescription);

    /* perspectiveRemovePixelPerCell */
    auto perspectiveRemovePixelPerCellDescription = rcl_interfaces::msg::ParameterDescriptor{};
    perspectiveRemovePixelPerCellDescription.name = "Number of bits (per dimension) for each cell of the marker when removing the perspective";
    auto perspectiveRemovePixelPerCellRange = rcl_interfaces::msg::IntegerRange();
    perspectiveRemovePixelPerCellRange.from_value = 1;
    perspectiveRemovePixelPerCellRange.to_value = std::numeric_limits<int>::max();
    perspectiveRemovePixelPerCellDescription.integer_range = {perspectiveRemovePixelPerCellRange};
    detectorParams->perspectiveRemovePixelPerCell = this->declare_parameter("perspectiveRemovePixelPerCell", 8, perspectiveRemovePixelPerCellDescription);

    /* polygonalApproxAccuracyRate */
    auto polygonalApproxAccuracyRateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    polygonalApproxAccuracyRateDescription.name = "Width of the margin of pixels on each cell not considered for the determination of the cell bit. Represents the rate respect to the total size of the cell, i.e. perpectiveRemovePixelPerCell";
    auto polygonalApproxAccuracyRateRange = rcl_interfaces::msg::FloatingPointRange();
    polygonalApproxAccuracyRateRange.from_value = 0.0;
    polygonalApproxAccuracyRateRange.to_value = 1.0;
    polygonalApproxAccuracyRateDescription.floating_point_range = {polygonalApproxAccuracyRateRange};
    detectorParams->polygonalApproxAccuracyRate = this->declare_parameter("polygonalApproxAccuracyRate", 0.01, polygonalApproxAccuracyRateDescription); /* default 0.05 */

    parameter_callback_handle = this->add_on_set_parameters_callback(std::bind(&FiducialsNode::param_change_callback, this, std::placeholders::_1));

    // RCLCPP_INFO(this->get_logger(), "Aruco detection ready");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FiducialsNode>());
    rclcpp::shutdown();
    return 0;
}
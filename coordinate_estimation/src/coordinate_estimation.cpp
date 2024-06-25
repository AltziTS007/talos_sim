#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "position_msgs/ObjectPosition.h"
#include "position_msgs/ObjectPositions.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <tf/transform_broadcaster.h>

darknet_ros_msgs::BoundingBox boundingBox; 
darknet_ros_msgs::BoundingBoxes boundingBoxesResults;
std::vector<darknet_ros_msgs::BoundingBox> v;
position_msgs::ObjectPositions objectpositionresults;
ros::Publisher PositionPublisher;

cv_bridge::CvImagePtr cv_ptr;
cv::Mat depthImage(480, 640, CV_16UC1);

double fx = 462.13;
double fy = 462.13;
double cx = 320.0;
double cy = 240.0;

int x_c = 0;
int y_c = 0;
double dist = 0;
double Xtarget = 0;
double Ytarget = 0;
double Ztarget = 0;

tf::TransformBroadcaster *br;

void bb_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

    v = msg->bounding_boxes;

    objectpositionresults.header.stamp = ros::Time::now();
    objectpositionresults.header.frame_id = "camera_depth_optical_frame";

    std::for_each(v.begin(), v.end(), [](const darknet_ros_msgs::BoundingBox& x){
        position_msgs::ObjectPosition objectPosition;
        ROS_INFO_STREAM("Processing bounding box: " << x);

        x_c = ((int) x.xmin + (int) x.xmax) / 2;
        y_c = ((int) x.ymin + (int) x.ymax) / 2;

        if (x_c < 0 || x_c >= depthImage.cols || y_c < 0 || y_c >= depthImage.rows) {
            ROS_WARN("Bounding box center out of depth image bounds");
            return;
        }

        dist = depthImage.at<unsigned short>(y_c, x_c);
        if (dist == 0) {
            ROS_WARN("Depth data missing for bounding box center");
            return;
        }

        objectPosition.Class = x.Class;
        objectPosition.x = dist * (x_c - cx) / fx;
        objectPosition.y = dist * (y_c - cy) / fy;
        objectPosition.z = dist;
        objectpositionresults.object_positions.push_back(objectPosition);

        // Create a transform for the detected object
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(objectPosition.x / 1000.0, objectPosition.y / 1000.0, objectPosition.z / 1000.0)); // Convert mm to meters
        tf::Quaternion q;
        q.setRPY(0, 0, 0); // No rotation
        transform.setRotation(q);

        // Broadcast the transform
        std::string object_frame_id = "object_" + x.Class;
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", object_frame_id));
    });

    PositionPublisher.publish(objectpositionresults);
    objectpositionresults.object_positions.clear();
}

void di_callback(const sensor_msgs::Image::ConstPtr& msg){
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("CV Bridge Exception: %s", ex.what());
        return;
    }
    depthImage = cv_ptr->image;
}

void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    fx = msg->K[0];
    cx = msg->K[2];
    fy = msg->K[4];
    cy = msg->K[5];
    ROS_INFO("Camera Info: fx: %lf, fy: %lf, cx: %lf, cy: %lf", fx, fy, cx, cy);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Coordinate_Estimation");
    ros::NodeHandle n;

    tf::TransformBroadcaster broadcaster;
    br = &broadcaster;

    ros::Subscriber camera_parameters = n.subscribe("/camera/color/camera_info", 10, camera_info_callback);
    ros::Subscriber Depth_image = n.subscribe("/camera/depth/image_rect_raw", 10, di_callback);
    ros::Subscriber Inference_result = n.subscribe("/darknet_ros/bounding_boxes", 10, bb_callback);
    PositionPublisher = n.advertise<position_msgs::ObjectPositions>("/objects_position/message", 10);
    ros::spin();

    return 0;
}

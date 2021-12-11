#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class octomap_frame_converter {
public:
  octomap_frame_converter()
    : tfListener_(tfBuffer_)
  {
    ros::NodeHandle nh, pnh("~");
    octomapSub_ = pnh.subscribe("input", 1, &octomap_frame_converter::octomapCallback, this);
    octomapPub_ = pnh.advertise<octomap_msgs::OctomapWithPose>("output",10);

    pnh.param<std::string>("target_frame", targetFrame_, "/odom");
  }

protected:
  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    octomap_msgs::OctomapWithPose msgOut;

    try{
      geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform(this->targetFrame_, msg->header.frame_id, msg->header.stamp, ros::Duration(3));
      msgOut.origin.position.x = transform.transform.translation.x;
      msgOut.origin.position.y = transform.transform.translation.y;
      msgOut.origin.position.z = transform.transform.translation.z;
      msgOut.origin.orientation.x = transform.transform.rotation.x;
      msgOut.origin.orientation.y = transform.transform.rotation.y;
      msgOut.origin.orientation.z = transform.transform.rotation.z;
      msgOut.origin.orientation.w = transform.transform.rotation.w;
    } catch (std::exception& ex) {
      ROS_ERROR_STREAM(ex.what());
      return;
    }

    msgOut.header.stamp = msg->header.stamp;
    msgOut.header.frame_id = this->targetFrame_;
    msgOut.octomap = *msg;

    this->octomapPub_.publish(msgOut);
  }

  ros::Subscriber octomapSub_;
  ros::Publisher octomapPub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  std::string targetFrame_;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_frame_converter");
  octomap_frame_converter c;

  ros::spin();

  return 0;
}

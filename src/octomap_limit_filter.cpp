#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class octomap_limit_filter {
public:
  octomap_limit_filter()
    : tfListener_(tfBuffer_)
  {
    ros::NodeHandle nh, pnh("~");
    octomapSub_ = pnh.subscribe("input", 1, &octomap_limit_filter::octomapCallback, this);
    octomapPub_ = pnh.advertise<octomap_msgs::Octomap>("output",10);

    pnh.param<std::string>("center_frame", centerFrame_, "");
    pnh.param<double>("radius", radius_, 100.0);
  }

protected:
  template<typename T>
  void applyLimitFilter(std::shared_ptr<T> octree, const octomap::point3d& min, const octomap::point3d& max){

    std::vector<std::pair<octomap::OcTreeKey, unsigned int> > toRemove;
    for(typename T::leaf_iterator it = octree->begin_leafs(),
          end=octree->end_leafs(); it!= end; ++it){
      // check if outside of bbx:
      octomap::OcTreeKey k = it.getKey();
      if  (k[0] < min.x() || k[1] < min.y() || k[2] < min.z()
           || k[0] > max.x() || k[1] > max.y() || k[2] > max.z()){
        toRemove.push_back(std::make_pair(k, it.getDepth()));
      }
    }

    for(int i=0;i<toRemove.size();i++){
      octree->deleteNode(toRemove[i].first, toRemove[i].second);
    }
  }

  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    std::shared_ptr<octomap::AbstractOcTree> absoctree = std::shared_ptr<octomap::AbstractOcTree>(octomap_msgs::msgToMap(*msg));
    if(!absoctree) return;
        std::shared_ptr<octomap::OcTree> octree = std::dynamic_pointer_cast<octomap::OcTree>(absoctree);
        std::shared_ptr<octomap::ColorOcTree> coloroctree = std::dynamic_pointer_cast<octomap::ColorOcTree>(absoctree);

    geometry_msgs::Vector3 center; center.x = 0; center.y = 0; center.z = 0;
    if(this->centerFrame_ != "" && this->centerFrame_ != msg->header.frame_id){
      try{
        geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform(msg->header.frame_id, this->centerFrame_, msg->header.stamp, ros::Duration(5));
        center = transform.transform.translation;
      } catch (std::exception& ex) {
        ROS_ERROR_STREAM(ex.what());
      }
    }
    octomap::point3d min(center.x - this->radius_,
                         center.y - this->radius_,
                         center.z - this->radius_);
    octomap::point3d max(center.x + this->radius_,
                         center.y + this->radius_,
                         center.z + this->radius_);

    if(octree){
      this->applyLimitFilter<octomap::OcTree>(octree, min, max);
      octomap_msgs::Octomap msgOut;
      if(msg->binary) octomap_msgs::binaryMapToMsg<octomap::OcTree>(*octree, msgOut);
      else octomap_msgs::fullMapToMsg<octomap::OcTree>(*octree, msgOut);
      this->octomapPub_.publish(msgOut);
    }else if(coloroctree){
      this->applyLimitFilter<octomap::ColorOcTree>(coloroctree, min, max);
      octomap_msgs::Octomap msgOut;
      if(msg->binary) octomap_msgs::binaryMapToMsg<octomap::ColorOcTree>(*coloroctree, msgOut);
      else octomap_msgs::fullMapToMsg<octomap::ColorOcTree>(*coloroctree, msgOut);
      this->octomapPub_.publish(msgOut);
    }
  }

  ros::Subscriber octomapSub_;
  ros::Publisher octomapPub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  std::string centerFrame_;
  double radius_;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_limit_filter");
  octomap_limit_filter c;

  ros::spin();

  return 0;
}

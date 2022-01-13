#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

class octomap_to_occupancygrid {
public:
  octomap_to_occupancygrid()
    : tfListener_(tfBuffer_)
  {
    ros::NodeHandle nh, pnh("~");
    octomapSub_ = pnh.subscribe("input", 1, &octomap_to_occupancygrid::octomapCallback, this);
    occupancyGridPub_ = pnh.advertise<nav_msgs::OccupancyGrid>("output",10);

    pnh.getParam("limit_frames", limitFrames_);
    pnh.param<double>("limit_min_z", limitMinZ_, 0.15);
    pnh.param<double>("limit_max_z", limitMaxZ_, 2.0);
  }

protected:
  template<typename T>
  void octoMapToOccupancyGrid(std::shared_ptr<T> octree, double limitFrameZ, nav_msgs::OccupancyGrid& msgOut){

    octomap::point3d min;
    {
      double x,y,z;
      octree->getMetricMin(x,y,z);
      min.x() = x; min.y() = y; min.z() = z;
    }
    octomap::point3d max;
    {
      double x,y,z;
      octree->getMetricMax(x,y,z);
      max.x() = x; max.y() = y; max.z() = z;
    }

    msgOut.info.map_load_time = msgOut.header.stamp;
    msgOut.info.resolution = octree->getResolution();
    msgOut.info.width = (max.x() - min.x()) / msgOut.info.resolution;
    msgOut.info.height = (max.y() - min.y()) / msgOut.info.resolution;
    msgOut.info.origin.position.x = min.x();
    msgOut.info.origin.position.y = min.y();
    msgOut.info.origin.position.z = 0.0;
    msgOut.info.origin.orientation.x = 0.0;
    msgOut.info.origin.orientation.y = 0.0;
    msgOut.info.origin.orientation.z = 0.0;
    msgOut.info.origin.orientation.w = 1.0;

    double resolution = octree->getResolution();
    double minx = min.x() + resolution/2;
    double miny = min.y() + resolution/2;
    double minz = min.z() + resolution/2;

    msgOut.data.resize(msgOut.info.width * msgOut.info.height, -1); // Unknown is -1.
    for(typename T::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it){
      double centerx = it.getX();
      double centery = it.getY();
      double centerz = it.getZ();
      double size = it.getSize();
      bool isOccupied = octree->isNodeOccupied(*it);

      if(centerz > limitFrameZ + this->limitMaxZ_ || centerz < limitFrameZ + this->limitMinZ_) continue;

      for(double x = centerx - size/2 + resolution/2; x < centerx + size/2; x+=resolution){
        int datax = (x - minx)/resolution;
        for(double y = centery - size/2 + resolution/2; y < centery + size/2; y+=resolution){
          int datay = (y - miny)/resolution;
          if(isOccupied) msgOut.data[datax + datay * msgOut.info.width] = 100;
          else if(msgOut.data[datax + datay * msgOut.info.width] == -1) msgOut.data[datax + datay * msgOut.info.width] = 0;
        }
      }
    }

  }

  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    std::shared_ptr<octomap::AbstractOcTree> absoctree = std::shared_ptr<octomap::AbstractOcTree>(octomap_msgs::msgToMap(*msg));
    if(!absoctree) return;

    double limitFrameZ = 1e10;
    if(this->limitFrames_.size() == 0) limitFrameZ = 0;
    else{
      for(int i=0;i<this->limitFrames_.size();i++){
        try{
          geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform(msg->header.frame_id, this->limitFrames_[i], msg->header.stamp, ros::Duration(5));
          limitFrameZ = std::min(limitFrameZ, transform.transform.translation.z);
        } catch (std::exception& ex) {
          ROS_ERROR_STREAM(ex.what());
          return;
        }
      }
    }

    nav_msgs::OccupancyGrid msgOut;
    msgOut.header = msg->header;

    if(msg->id == "OcTree"){
      std::shared_ptr<octomap::OcTree> octree = std::dynamic_pointer_cast<octomap::OcTree>(absoctree);
      if(octree){
        this->octoMapToOccupancyGrid<octomap::OcTree>(octree, limitFrameZ, msgOut);
        this->occupancyGridPub_.publish(msgOut);
      }
    }else if(msg->id == "ColorOcTree"){
      std::shared_ptr<octomap::ColorOcTree> coloroctree = std::dynamic_pointer_cast<octomap::ColorOcTree>(absoctree);
      if(coloroctree){
        this->octoMapToOccupancyGrid<octomap::ColorOcTree>(coloroctree, limitFrameZ, msgOut);
        this->occupancyGridPub_.publish(msgOut);
      }
    }

    absoctree->clear(); // destructor of OcTree does not free memory for internal data.
  }

  ros::Subscriber octomapSub_;
  ros::Publisher occupancyGridPub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  std::vector<std::string> limitFrames_;
  double limitMinZ_;
  double limitMaxZ_;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_to_occupancygrid");
  octomap_to_occupancygrid c;

  ros::spin();

  return 0;
}

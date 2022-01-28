#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>

class octomap_speckle_remover {
public:
  octomap_speckle_remover()
    : tfListener_(tfBuffer_),
      nh_(),
      pnh_("~")
  {
    octomapSub_ = pnh_.subscribe("input", 1, &octomap_speckle_remover::octomapCallback, this);
    octomapPub_ = pnh_.advertise<octomap_msgs::Octomap>("output",10);

    pnh_.param<int>("size", size_, 1);
  }

protected:
  template<typename T>
  int calcSize(const octomap::OcTreeKey& key, std::shared_ptr<T> octree, std::unordered_map<octomap::OcTreeKey,unsigned int,octomap::OcTreeKey::KeyHash>& hasArrived,std::unordered_map<octomap::OcTreeKey,unsigned int,octomap::OcTreeKey::KeyHash>& currentGroup){
    if(hasArrived.find(key) != hasArrived.end()) return 0;
    octomap::OcTreeNode* node = octree->search(key);
    if(!node) return 0;
    if(!octree->isNodeOccupied(node)) return 0;

    hasArrived[key] = 0;
    currentGroup[key] = 0;
    int size = 1;

    for(int x=-1;x<=1;x+=2){
      for(int y=-1;y<=1;y+=2){
        for(int z=-1;z<=1;z+=2){
          octomap::OcTreeKey neighborKey = key;
          neighborKey[0]+=x;
          neighborKey[1]+=y;
          neighborKey[2]+=z;
          size += this->calcSize(neighborKey,octree,hasArrived,currentGroup);
        }
      }
    }

    return size;
  }

  template<typename T>
  void applyFilter(std::shared_ptr<T> octree){
    std::unordered_map<octomap::OcTreeKey,unsigned int,octomap::OcTreeKey::KeyHash> hasArrived;
    std::unordered_map<octomap::OcTreeKey,unsigned int,octomap::OcTreeKey::KeyHash> toRemove;
    for(typename T::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it){
      if(hasArrived.find(it.getKey()) != hasArrived.end()) continue;

      std::unordered_map<octomap::OcTreeKey,unsigned int,octomap::OcTreeKey::KeyHash> currentGroup;
      int size = this->calcSize(it.getKey(),octree,hasArrived,currentGroup);
      if(size <= this->size_) {
        for(std::unordered_map<octomap::OcTreeKey,unsigned int,octomap::OcTreeKey::KeyHash>::iterator it = currentGroup.begin(); it != currentGroup.end(); it++){
          toRemove[it->first] = it->second;
        }
      }
    }
    for(std::unordered_map<octomap::OcTreeKey,unsigned int,octomap::OcTreeKey::KeyHash>::iterator it = toRemove.begin(); it != toRemove.end(); it++){
      //octree->deleteNode(it->first,it->second);
      octree->deleteNode(it->first);
    }

    octree->prune();
  }

  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    std::shared_ptr<octomap::AbstractOcTree> absoctree = std::shared_ptr<octomap::AbstractOcTree>(octomap_msgs::msgToMap(*msg));
    if(!absoctree) return;

    if(msg->id == "OcTree"){
      std::shared_ptr<octomap::OcTree> octree = std::dynamic_pointer_cast<octomap::OcTree>(absoctree);
      if(octree){
        this->applyFilter<octomap::OcTree>(octree);
        octomap_msgs::Octomap msgOut;
        if(msg->binary) octomap_msgs::binaryMapToMsg<octomap::OcTree>(*octree, msgOut);
        else octomap_msgs::fullMapToMsg<octomap::OcTree>(*octree, msgOut);
        msgOut.header = msg->header;
        this->octomapPub_.publish(msgOut);
      }
    }else if(msg->id == "ColorOcTree"){
      std::shared_ptr<octomap::ColorOcTree> coloroctree = std::dynamic_pointer_cast<octomap::ColorOcTree>(absoctree);
      if(coloroctree){
        this->applyFilter<octomap::ColorOcTree>(coloroctree);
        octomap_msgs::Octomap msgOut;
        if(msg->binary) octomap_msgs::binaryMapToMsg<octomap::ColorOcTree>(*coloroctree, msgOut);
        else octomap_msgs::fullMapToMsg<octomap::ColorOcTree>(*coloroctree, msgOut);
        msgOut.header = msg->header;
        this->octomapPub_.publish(msgOut);
      }
    }

    absoctree->clear(); // destructor of OcTree does not free memory for internal data.
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber octomapSub_;
  ros::Publisher octomapPub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  int size_;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_speckle_remover");
  octomap_speckle_remover c;

  ros::spin();

  return 0;
}

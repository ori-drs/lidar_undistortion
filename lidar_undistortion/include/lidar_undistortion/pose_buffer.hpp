#pragma once
#include <Eigen/Geometry>
#include <map>

class PoseBuffer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  using PoseHistory = std::map<uint64_t, Eigen::Isometry3d, std::less<uint64_t>,
  Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Isometry3d> > >;
  using Vector3d = Eigen::Vector3d;
  using Quaternion = Eigen::Quaterniond;

public:
  PoseBuffer() {}

  PoseBuffer(uint64_t buffer_size) : buffer_size_(buffer_size){

  }

  std::string toString(){
    std::stringstream ss;
    Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
    for(const auto& kv : pose_history_){
      ss << "[ " << kv.first << "] " << std::endl << kv.second.matrix().format(fmt) << std::endl;
    }
    return ss.str();
  }

  void setBufferSize(uint64_t buffer_size) {
    buffer_size_ = buffer_size;
  }

  uint64_t getBufferSize() {
    return buffer_size_;
  }

  bool empty() {
    return pose_history_.empty();
  }

  bool getInterpolatedPose(const uint64_t &nsec,
                                   Eigen::Isometry3d& pose) const;

  bool canInterpolate(uint64_t nsec);

  bool hasPose(uint64_t nsec) {
    return pose_history_.find(nsec) != pose_history_.end();
  }

  Eigen::Isometry3d getPose(uint64_t nsec){
    if(hasPose(nsec)){
      return pose_history_[nsec];
    }
    return Eigen::Isometry3d::Identity();
  }

  void addPose(uint64_t nsec, const Eigen::Isometry3d& pose);

  uint64_t startTime(){
    if(pose_history_.empty()){
      return 0;
    }
    return pose_history_.begin()->first;
  }

  uint64_t endTime(){
    if(pose_history_.empty()){
      return 0;
    }
    return pose_history_.rbegin()->first;
  }

  size_t size(){
    return pose_history_.size();
  }

private:
  PoseHistory pose_history_;
  uint64_t buffer_size_ =   100000000000;
};

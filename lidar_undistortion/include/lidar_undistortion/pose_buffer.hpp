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

  virtual std::string toString(){
    std::stringstream ss;
    Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
    for(const auto& kv : pose_history_){
      ss << "[ " << kv.first << "] " << std::endl << kv.second.matrix().format(fmt) << std::endl;
    }
    return ss.str();
  }

  virtual void setBufferSize(uint64_t buffer_size) {
    buffer_size_ = buffer_size;
  }

  virtual uint64_t getBufferSize() {
    return buffer_size_;
  }

  virtual bool empty() {
    return pose_history_.empty();
  }

  virtual bool getInterpolatedPose(const uint64_t &nsec,
                                   Eigen::Isometry3d& pose) const;

  virtual bool canInterpolate(uint64_t nsec);

  virtual bool hasPose(uint64_t nsec) {
    return pose_history_.find(nsec) != pose_history_.end();
  }

  virtual Eigen::Isometry3d getPose(uint64_t nsec){
    if(hasPose(nsec)){
      return pose_history_[nsec];
    }
    return Eigen::Isometry3d::Identity();
  }

  virtual void addPose(uint64_t nsec, const Eigen::Isometry3d& pose);

  virtual uint64_t startTime(){
    if(pose_history_.empty()){
      return 0;
    }
    return pose_history_.begin()->first;
  }

  virtual uint64_t endTime(){
    if(pose_history_.empty()){
      return 0;
    }
    return pose_history_.rbegin()->first;
  }

  virtual size_t size(){
    return pose_history_.size();
  }

private:
  PoseHistory pose_history_;
  uint64_t buffer_size_ =   100000000000;
};
